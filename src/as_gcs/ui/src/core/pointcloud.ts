/**
 * sensor_msgs/PointCloud2 字段解析 → Float32Array
 *
 * 从 @foxglove/rosmsg-serialization 解出来的 data 是 Uint8Array（原始字节），
 * fields 描述每个字段在 point_step 中的偏移与类型。
 */

/** sensor_msgs/PointField 的 datatype 取值 */
const INT8 = 1;
const UINT8 = 2;
const INT16 = 3;
const UINT16 = 4;
const INT32 = 5;
const UINT32 = 6;
const FLOAT32 = 7;
const FLOAT64 = 8;

export interface PointField {
  name: string;
  offset: number;
  datatype: number;
  count: number;
}

export interface PointCloud2Msg {
  header: { frame_id: string; stamp: { sec: number; nsec: number } };
  height: number;
  width: number;
  fields: PointField[];
  is_bigendian: boolean;
  point_step: number;
  row_step: number;
  data: Uint8Array;
  is_dense: boolean;
}

export interface ParsedCloud {
  /** xyz 交错，长度 = count * 3 */
  positions: Float32Array;
  /** 可选标量（用于着色），长度 = count */
  scalar: Float32Array | undefined;
  scalarName: string;
  count: number;
}

const EMPTY: ParsedCloud = {
  positions: new Float32Array(0),
  scalar: undefined,
  scalarName: "",
  count: 0,
};

/**
 * 返回一个按 (base, offset) 读标量的函数。
 *
 * 注意：base 必须通过【每次调用】传入，不能写成闭包捕获。
 * 曾经写成 scalarReader(dv, 0, TYPE) 然后在循环里 gx(fx.offset)，
 * 结果所有点都读的同一偏移，表现为「整片点云只剩一个点」。
 */
type ScalarGetter = (base: number, offset: number, isBE: boolean) => number;

function scalarReader(dv: DataView, datatype: number): ScalarGetter | undefined {
  switch (datatype) {
    case INT8:
      return (b, o) => dv.getInt8(b + o);
    case UINT8:
      return (b, o) => dv.getUint8(b + o);
    case INT16:
      return (b, o, be) => dv.getInt16(b + o, !be);
    case UINT16:
      return (b, o, be) => dv.getUint16(b + o, !be);
    case INT32:
      return (b, o, be) => dv.getInt32(b + o, !be);
    case UINT32:
      return (b, o, be) => dv.getUint32(b + o, !be);
    case FLOAT32:
      return (b, o, be) => dv.getFloat32(b + o, !be);
    case FLOAT64:
      return (b, o, be) => dv.getFloat64(b + o, !be);
    default:
      return undefined;
  }
}

/**
 * 解析一帧点云。
 * intensity 优先，其次 ring，都没有就不着色。
 */
export function parsePointCloud2(msg: PointCloud2Msg): ParsedCloud {
  if (!msg || !msg.fields || !msg.data) return EMPTY;

  const fx = msg.fields.find((f) => f.name === "x");
  const fy = msg.fields.find((f) => f.name === "y");
  const fz = msg.fields.find((f) => f.name === "z");
  if (!fx || !fy || !fz) return EMPTY;

  const scalarField =
    msg.fields.find((f) => f.name === "intensity") ?? msg.fields.find((f) => f.name === "ring");

  const pointStep = msg.point_step;
  if (!pointStep || pointStep <= 0) return EMPTY;

  const total = Math.floor(msg.data.byteLength / pointStep);
  if (total <= 0) return EMPTY;

  const count = Math.max(1, msg.width) * Math.max(1, msg.height);
  const n = Math.min(total, count > 0 ? count : total);

  const positions = new Float32Array(n * 3);
  const scalar = scalarField ? new Float32Array(n) : undefined;

  // data 可能不是 8 字节对齐；DataView 需要显式给 byteOffset
  const buf = msg.data.buffer as ArrayBuffer;
  const dv = new DataView(buf, msg.data.byteOffset, msg.data.byteLength);
  const be = msg.is_bigendian;

  const gx = scalarReader(dv, fx.datatype)!;
  const gy = scalarReader(dv, fy.datatype)!;
  const gz = scalarReader(dv, fz.datatype)!;
  const gs = scalarField ? scalarReader(dv, scalarField.datatype) : undefined;

  let valid = 0;
  for (let i = 0; i < n; i++) {
    const base = i * pointStep;
    const x = gx(base, fx.offset, be);
    const y = gy(base, fy.offset, be);
    const z = gz(base, fz.offset, be);
    // 非法点（NaN/inf）在 is_dense=false 的帧里常见，跳过
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;

    positions[valid * 3] = x;
    positions[valid * 3 + 1] = y;
    positions[valid * 3 + 2] = z;
    if (scalar && gs) scalar[valid] = gs(base, scalarField!.offset, be);
    valid++;
  }

  if (valid === 0) return EMPTY;

  return {
    positions: valid === n ? positions : positions.subarray(0, valid * 3),
    scalar: scalar && valid === n ? scalar : scalar?.subarray(0, valid),
    scalarName: scalarField?.name ?? "",
    count: valid,
  };
}

/** 体素降采样（前端兜底用；正常应在 ROS 侧做） */
export function voxelDownsample(
  cloud: ParsedCloud,
  voxelSize: number,
): ParsedCloud {
  if (voxelSize <= 0 || cloud.count === 0) return cloud;
  const inv = 1 / voxelSize;
  const seen = new Set<number>();
  const outPos = new Float32Array(cloud.count * 3);
  const outScalar = cloud.scalar ? new Float32Array(cloud.count) : undefined;
  let w = 0;

  for (let i = 0; i < cloud.count; i++) {
    const x = cloud.positions[i * 3];
    const y = cloud.positions[i * 3 + 1];
    const z = cloud.positions[i * 3 + 2];
    // 17 bit × 3 打包进整数域，避免字符串 key 的开销
    const kx = Math.floor(x * inv) + 65536;
    const ky = Math.floor(y * inv) + 65536;
    const kz = Math.floor(z * inv) + 65536;
    const key = kx * 65536 * 2 + ky * 2 + kz;
    if (seen.has(key)) continue;
    seen.add(key);
    outPos[w * 3] = x;
    outPos[w * 3 + 1] = y;
    outPos[w * 3 + 2] = z;
    if (outScalar && cloud.scalar) outScalar[w] = cloud.scalar[i];
    w++;
  }

  return {
    positions: outPos.subarray(0, w * 3),
    scalar: outScalar?.subarray(0, w),
    scalarName: cloud.scalarName,
    count: w,
  };
}
