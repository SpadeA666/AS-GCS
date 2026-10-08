/**
 * 控制面板：任务级交互键
 *
 * 设计边界（重要）：
 *   地面站只发任务级命令。避障、轨迹跟踪、飞控安全逻辑全部留在 as_controller 里。
 *   这里不直接碰 /mavros/setpoint_raw/local。
 *
 * 所有按键都走 gcs_gateway 的服务。网关没起来时按钮置灰并给出提示，
 * 不做"看起来能点其实没反应"的假交互。
 *
 * 颜色编码（按危险等级）：
 *   红 = 不可逆或紧急（急停、降落）
 *   橙 = 有物理动作但可逆（激光、投放相关）
 *   蓝 = 普通状态切换
 */
import { useCallback, useState } from "react";
import type { FoxgloveConnection } from "../core/FoxgloveConnection.ts";
import type { Geofence } from "../core/types.ts";
import { sortPolygon } from "../core/geofence.ts";
import { NumberField } from "./NumberField";

export const SVC = {
  takeoff: "/gcs/takeoff",
  land: "/gcs/land",
  flyUp: "/gcs/fly_up",
  flyDown: "/gcs/fly_down",
  goToPx4: "/gcs/go_to_px4",
  setPlanner: "/gcs/set_planner",
  setNavMode: "/gcs/set_nav_mode",
  startFollow: "/gcs/start_follow",
  stopFollow: "/gcs/stop_follow",
  alignTarget: "/gcs/align_target",
  setActuator: "/gcs/set_actuator",
  setGeofence: "/gcs/set_geofence",
  emergency: "/gcs/emergency_stop",
  camera: "/gcs/camera_control",
  yolo: "/gcs/yolo_control",
} as const;

export type Planner = "ego" | "super";
export type CameraMode = "off" | "mono" | "d435i" | "both";

const LABEL_CAM: Record<CameraMode, string> = {
  off: "关闭",
  mono: "单目",
  d435i: "D435i",
  both: "混合",
};

interface Props {
  conn: FoxgloveConnection | undefined;
  services: Set<string>;
  planner: Planner;
  onPlannerChange: (p: Planner) => void;
  cameraMode: CameraMode;
  onCameraModeChange: (m: CameraMode) => void;
  onLog: (msg: string) => void;
  /** 安全区（提升到父层，2D 画布与这里的控件共用同一份） */
  geofence?: Geofence;
  onGeofenceChange?: (g: Geofence) => void;
  /** 「启用并下发」/「关闭」时触发，由父层调 /gcs/set_geofence */
  onApplyGeofence?: (g: Geofence) => void;
  /** 安全区编辑模式：开启后在 2D 画布上加点/拖点/删点 */
  fenceMode?: boolean;
  onFenceModeChange?: (v: boolean) => void;
  /**
   * 当前飞行高度（ENU 的 z，米）。
   * 上升/下降填的是【目标高度】，而网关的 fly_up/fly_down 接的是【增量 delta】，
   * 所以要拿当前高度换算：上升 delta = 目标 − 当前，下降 delta = 当前 − 目标。
   */
  currentZ?: number;
}

export function ControlPanel({
  conn,
  services,
  planner,
  onPlannerChange,
  cameraMode,
  onCameraModeChange,
  onLog,
  geofence,
  onGeofenceChange,
  onApplyGeofence,
  fenceMode = false,
  onFenceModeChange,
  currentZ,
}: Props) {
  const [busy, setBusy] = useState<string | undefined>(undefined);
  const [yoloOn, setYoloOn] = useState(false);
  const [servo5, setServo5] = useState(50);
  const [servo6, setServo6] = useState(50);
  const [laserOn, setLaserOn] = useState(false);
  /** 起飞目标高度（m）——以前写死 1.0 */
  const [takeoffHeight, setTakeoffHeight] = useState(1.0);
  /** 上升/下降的目标高度（m，绝对值）——以前是增量 delta，写死 0.5 */
  const [climbTarget, setClimbTarget] = useState(1.5);
  /**
   * 规划器打点时 Z / Yaw 的控制来源（true = 用打点填的值）。
   * 默认两个都关 = 全交给规划器（NAV_FULL），即改动前的行为。
   */
  const [zCtrl, setZCtrl] = useState(false);
  const [yawCtrl, setYawCtrl] = useState(false);

  const has = (svc: string) => services.has(svc);

  const call = useCallback(
    async (svc: string, req: unknown, label: string) => {
      if (!conn) {
        onLog(`${label}: 未连接`);
        return;
      }
      if (!has(svc)) {
        onLog(`${label}: 网关服务 ${svc} 不存在（gcs_gateway 还没起）`);
        return;
      }
      setBusy(svc);
      try {
        const res = (await conn.callService(svc, req)) as Record<string, unknown>;
        const ok = res?.success ?? true;
        const msg = (res?.message as string) ?? "";
        onLog(`${label}: ${ok ? "成功" : "失败"}${msg ? " — " + msg : ""}`);
      } catch (e) {
        onLog(`${label}: 调用异常 — ${String(e)}`);
      } finally {
        setBusy(undefined);
      }
    },
    [conn, services, onLog],
  );

  /**
   * 切换规划器打点的 Z / Yaw 控制来源。
   *
   * nav_mode 的实际位语义（按 ASNAV 里 use_z / use_yaw 的定义反推）：
   *   use_z   = (m == 0 || m == 1)   → bit1 置位 ⇒ Z 手动
   *   use_yaw = (m == 0 || m == 2)   → bit0 置位 ⇒ Yaw 手动
   * 注意头文件里写的“bit0 = Z, bit1 = Yaw”与代码行为是相反的，以代码为准。
   *
   *   Z规划器+Yaw规划器 → 0 NAV_FULL
   *   Z规划器+Yaw手动   → 1 NAV_Z_ONLY
   *   Z手动+Yaw规划器   → 2 NAV_YAW_ONLY
   *   Z手动+Yaw手动     → 3 NAV_LEVEL
   */
  const setNavAxes = (yawManual: boolean, zManual: boolean) => {
    setYawCtrl(yawManual);
    setZCtrl(zManual);
    // bit0 = Yaw 手动，bit1 = Z 手动
    const m = (yawManual ? 1 : 0) | (zManual ? 2 : 0);
    void call(
      SVC.setNavMode,
      { nav_mode: m },
      `导航轴模式 → ${m}（机头${yawManual ? "手动" : "规划器"} / 高度${zManual ? "手动" : "规划器"}）`,
    );
  };

  const disabled = !conn || busy !== undefined;
  const gwMissing = !has(SVC.land);

  return (
    <div className="ctrl">
      {gwMissing && (
        <div className="warn">
          网关服务未就绪（<code>gcs_gateway</code> 还没启动），以下按键暂不可用。
          现在只有 2D 打点的「规划器点」是可直接发出去的。
        </div>
      )}

      {/* 急停：点按触发，视觉上必须最显眼 */}
      <button
        className="estop"
        disabled={disabled}
        onClick={() => void call(SVC.emergency, { action: "hover" }, "急停（悬停）")}
        title="立即触发：停止规划器目标 + 位置锁存悬停"
      >
        急停
      </button>

      <div className="grp">
        <div className="grp-title">飞行</div>

        <div className="row">
          <button
            disabled={disabled}
            onClick={() => call(SVC.takeoff, { height: takeoffHeight }, `起飞到 ${takeoffHeight.toFixed(2)}m`)}
          >
            起飞
          </button>
          <span className="unit">到</span>
          <NumberField
            value={takeoffHeight}
            onValueChange={setTakeoffHeight}
            step="0.1"
            min={0.3}
            max={3}
            title="起飞目标高度（m）"
          />
          <span className="unit">m</span>
          <button
            className="btn-danger"
            disabled={disabled}
            onClick={() => call(SVC.land, {}, "降落")}
          >
            降落
          </button>
        </div>

        <div className="row">
          <button
            disabled={disabled}
            onClick={() => {
              if (currentZ === undefined) {
                onLog("上升失败：还没收到无人机位姿，无法换算目标高度");
                return;
              }
              const delta = climbTarget - currentZ;
              call(
                SVC.flyUp,
                { delta },
                `上升到 ${climbTarget.toFixed(2)}m（Δ${delta.toFixed(2)}）`,
              );
            }}
          >
            上升
          </button>
          <button
            disabled={disabled}
            onClick={() => {
              if (currentZ === undefined) {
                onLog("下降失败：还没收到无人机位姿，无法换算目标高度");
                return;
              }
              const delta = currentZ - climbTarget;
              call(
                SVC.flyDown,
                { delta },
                `下降到 ${climbTarget.toFixed(2)}m（Δ${delta.toFixed(2)}）`,
              );
            }}
          >
            下降
          </button>
          <span className="unit">到</span>
          <NumberField
            value={climbTarget}
            onValueChange={setClimbTarget}
            step="0.1"
            min={0.2}
            max={3}
            title="上升/下降的目标高度（m，绝对值；与起飞同一种填写方式）"
          />
          <span className="unit">m</span>
        </div>

        <div className="hint">
          起飞/上升/下降都直接填目标高度（绝对值，米）；当前高度 Z 不够时按真实位姿换算（受 SUPER 天花板 1.8m / 地面 −0.6m 限制）
        </div>
      </div>

      <div className="grp">
        <div className="grp-title">规划器</div>
        <div className="row">
          <button
            className={planner === "ego" ? "on" : ""}
            onClick={() => onPlannerChange("ego")}
            title="切换会清空当前航点队列"
          >
            EGO
          </button>
          <button
            className={planner === "super" ? "on" : ""}
            onClick={() => onPlannerChange("super")}
            title="切换会清空当前航点队列"
          >
            SUPER
          </button>
        </div>

        {/* 规划器打点时，Z / Yaw 交给谁。位置按 bit 语义组合成 nav_mode：bit0=Z, bit1=Yaw */}
        <div className="row">
          <button
            className={yawCtrl ? "on" : ""}
            disabled={disabled}
            onClick={() => setNavAxes(!yawCtrl, zCtrl)}
            title="开启：打点填的机头角度生效；关闭：由规划器决定"
          >
            机头 {yawCtrl ? "手动" : "规划器"}
          </button>
          <button
            className={zCtrl ? "on" : ""}
            disabled={disabled}
            onClick={() => setNavAxes(yawCtrl, !zCtrl)}
            title="开启：打点填的高度生效；关闭：由规划器决定"
          >
            高度 {zCtrl ? "手动" : "规划器"}
          </button>
        </div>
        <div className="hint">
          仅影响「规划器点」；PX4 点始终按填的 Z/Yaw 执行
          （nav_mode {(yawCtrl ? 1 : 0) | (zCtrl ? 2 : 0)}）
        </div>
      </div>

      <div className="grp">
        <div className="grp-title">摄像头</div>
        <div className="row">
          {(["off", "mono", "d435i", "both"] as CameraMode[]).map((m) => (
            <button
              key={m}
              className={cameraMode === m ? "on" : ""}
              disabled={disabled}
              onClick={() => {
                onCameraModeChange(m);
                void call(SVC.camera, { mode: m }, `摄像头 → ${LABEL_CAM[m]}`);
              }}
            >
              {LABEL_CAM[m]}
            </button>
          ))}
        </div>
        <div className="row">
          <span className="lbl">YOLO</span>
          <button
            className={yoloOn ? "on" : ""}
            disabled={disabled}
            onClick={() => {
              const next = !yoloOn;
              setYoloOn(next);
              void call(SVC.yolo, { enable: next }, `YOLO ${next ? "开启" : "关闭"}`);
            }}
          >
            {yoloOn ? "已开启" : "已关闭"}
          </button>
        </div>
        <div className="hint">YOLO 跑哪个 launch 自动跟随摄像头模式</div>
      </div>

      <div className="grp">
        <div className="grp-title">目标</div>
        <div className="row">
          <button
            disabled={disabled}
            onClick={() => call(SVC.startFollow, { target_class: "", mode: 0 }, "开始跟随")}
          >
            开始跟随
          </button>
          <button disabled={disabled} onClick={() => call(SVC.stopFollow, {}, "停止跟随")}>
            停止跟随
          </button>
          <button
            className="btn-warn"
            disabled={disabled}
            onClick={() => call(SVC.alignTarget, { target_class: "" }, "目标对齐")}
          >
            目标对齐
          </button>
        </div>
      </div>

      <div className="grp">
        <div className="grp-title">执行机构</div>
        <div className="row">
          <span className="lbl">舵机 M5</span>
          <input
            type="range"
            min={0}
            max={100}
            value={servo5}
            disabled={disabled}
            onChange={(e) => setServo5(parseInt(e.target.value, 10))}
            onPointerUp={() =>
              call(SVC.setActuator, { channel_5: servo5, channel_6: -1, channel_7: -1 }, "舵机 M5")
            }
          />
          <span className="num-val">{servo5}</span>
        </div>
        <div className="row">
          <span className="lbl">舵机 M6</span>
          <input
            type="range"
            min={0}
            max={100}
            value={servo6}
            disabled={disabled}
            onChange={(e) => setServo6(parseInt(e.target.value, 10))}
            onPointerUp={() =>
              call(SVC.setActuator, { channel_5: -1, channel_6: servo6, channel_7: -1 }, "舵机 M6")
            }
          />
          <span className="num-val">{servo6}</span>
        </div>
        <div className="row">
          <span className="lbl">激光 M7</span>
          <button
            className={laserOn ? "btn-warn on" : "btn-warn"}
            disabled={disabled}
            onPointerDown={() => {
              setLaserOn(true);
              void call(
                SVC.setActuator,
                { channel_5: -1, channel_6: -1, channel_7: 100 },
                "激光点亮",
              );
            }}
            onPointerUp={() => {
              setLaserOn(false);
              void call(SVC.setActuator, { channel_5: -1, channel_6: -1, channel_7: 0 }, "激光熄灭");
            }}
            onPointerLeave={() => {
              if (!laserOn) return;
              setLaserOn(false);
              void call(SVC.setActuator, { channel_5: -1, channel_6: -1, channel_7: 0 }, "激光熄灭");
            }}
          >
            {laserOn ? "● 点亮中" : "按住点亮"}
          </button>
        </div>
      </div>

      <div className="grp">
        <div className="grp-title">安全区</div>

        {/* 绘制：开启后在 2D 画布上左键加点 / 拖顶点移动 / 右键顶点删除 */}
        <div className="row">
          <button
            className={fenceMode ? "on" : ""}
            onClick={() => onFenceModeChange?.(!fenceMode)}
            title="开启后在 2D 画布上：左键空白处加点 / 拖顶点移动 / 右键顶点删除"
          >
            {fenceMode ? "● 编辑中" : "绘制边界"}
          </button>
          <span className="unit">{geofence?.polygon.length ?? 0} 顶点</span>
        </div>

        {/* 高度上下限：安全区的 Z 范围 */}
        <div className="row">
          <span className="unit">Z</span>
          <NumberField
            value={geofence?.zMin ?? 0}
            onValueChange={(v) => geofence && onGeofenceChange?.({ ...geofence, zMin: v })}
            step="0.1"
            disabled={!geofence}
            title="安全区高度下限（m）"
          />
          <span className="unit">~</span>
          <NumberField
            value={geofence?.zMax ?? 0}
            onValueChange={(v) => geofence && onGeofenceChange?.({ ...geofence, zMax: v })}
            step="0.1"
            disabled={!geofence}
            title="安全区高度上限（m）"
          />
          <span className="unit">m</span>
        </div>

        {/* 下发 / 关闭 / 清除 */}
        <div className="row">
          <button
            className={geofence?.enabled ? "on" : ""}
            disabled={disabled || (geofence?.polygon.length ?? 0) < 3}
            onClick={() => {
              if (!geofence) return;
              // 下发前按极角排序并写回 state：
              // 一是让网关收到的是简单多边形（不会自交），
              // 二是 3D 那边直接用 state 画棱柱，也就跟着正了。
              // 存储顺序变了不影响拖拽——vertexAt() 按坐标找最近顶点，不依赖索引。
              const sorted = sortPolygon(geofence.polygon);
              onGeofenceChange?.({ ...geofence, polygon: sorted, enabled: true });
              onApplyGeofence?.({ ...geofence, polygon: sorted, enabled: true });
              // 下发完就退出编辑模式：否则下一个左键点击会被当成“加顶点”，
              // 想打点反而又把安全区改掉。退出后固若金汤，直接打点即可。
              onFenceModeChange?.(false);
            }}
            title={
              (geofence?.polygon.length ?? 0) < 3
                ? "至少需要 3 个顶点"
                : "下发到网关（ROS 侧强制校验，区外不允许飞行）"
            }
          >
            启用并下发
          </button>
          {geofence?.enabled && (
            <button
              className="btn-danger"
              disabled={disabled}
              onClick={() => {
                onApplyGeofence?.({ ...geofence, enabled: false });
                onFenceModeChange?.(false);   // 关闭后同样退出编辑，便于直接打点
              }}
            >
              关闭
            </button>
          )}
          <button
            disabled={!geofence || (geofence?.polygon.length ?? 0) === 0}
            onClick={() => geofence && onGeofenceChange?.({ ...geofence, polygon: [] })}
            title="清空多边形顶点（不会自动下发，需再点一次「启用并下发」）"
          >
            清除
          </button>
        </div>

        <div className="hint">
          {geofence?.enabled
            ? `已启用（Z ${geofence.zMin.toFixed(1)} ~ ${geofence.zMax.toFixed(1)} m），越界会被 ROS 侧拒绝`
            : "未启用；绘制后点「启用并下发」才生效。校验在 ROS 侧强制，前端只是可视化"}
        </div>
      </div>
    </div>
  );
}
