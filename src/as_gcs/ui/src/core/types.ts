/** 跨面板共享的数据结构 */

/**
 * 安全区：2D 多边形 + 高度上下限（棱柱）。
 *
 * 校验必须在 ROS 侧（gcs_gateway）强制，前端这份只是可视化和编辑用——
 * 任何人都能改前端代码绕过校验，那不算约束。
 */
export interface Geofence {
  enabled: boolean;
  /** 多边形顶点（ROS 的 XY 平面） */
  polygon: { x: number; y: number }[];
  zMin: number;
  zMax: number;
}

/** 无人机位姿（3D 模型用） */
export interface DronePose {
  x: number;
  y: number;
  z: number;
  qx: number;
  qy: number;
  qz: number;
  qw: number;
}
