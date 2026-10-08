/**
 * 3D 视图面板：点云（原生 / 膨胀两个开关）+ 四旋翼模型 + 轨迹 + 路径
 *
 * 话题不再全部列出 —— 只暴露两个有实际意义的开关：
 *   原生点云  → /super_cloud（退化到 /cloud_registered）
 *   膨胀点云  → /fsm_node/rog_map/inf_occ（SUPER 的 ROG Inflated）
 *
 * 所有开关状态由父组件持有，切到 2D 再回来不会重置。
 */
import { useEffect, useMemo, useRef, useState } from "react";
import type { FoxgloveConnection } from "../core/FoxgloveConnection.ts";
import type { DronePose, Geofence } from "../core/types.ts";
import { parsePointCloud2, voxelDownsample, type PointCloud2Msg } from "../core/pointcloud.ts";
import { SceneManager } from "../renderers/SceneManager.ts";

export type { DronePose };

interface Props {
  conn: FoxgloveConnection | undefined;
  /** 原生点云话题（父组件按可用话题挑好） */
  rawTopic: string;
  /** 膨胀点云话题 */
  inflatedTopic: string;
  showRaw: boolean;
  showInflated: boolean;
  onShowRawChange: (v: boolean) => void;
  onShowInflatedChange: (v: boolean) => void;
  trajectory: number[];
  planPath: number[];
  /** SUPER 期望轨迹（rviz 的 ExpTraj），扁平 xyz */
  expTraj: number[];
  expTrajColor: string;
  drone?: DronePose;
  /** 安全区（3D 只负责显示棱柱，编辑在 2D） */
  geofence?: Geofence;
}

export function ThreeDPanel({
  conn,
  rawTopic,
  inflatedTopic,
  showRaw,
  showInflated,
  onShowRawChange,
  onShowInflatedChange,
  trajectory,
  planPath,
  expTraj,
  expTrajColor,
  drone,
  geofence,
}: Props) {
  const hostRef = useRef<HTMLDivElement>(null);
  const mgrRef = useRef<SceneManager | undefined>(undefined);
  const unsubRef = useRef(new Map<string, () => void>());
  const framedRef = useRef(false);

  const [stats, setStats] = useState<Record<string, { pts: number; hz: number }>>({});

  // 当前应订阅的话题集合
  const wanted = useMemo(() => {
    const list: string[] = [];
    if (showRaw && rawTopic) list.push(rawTopic);
    if (showInflated && inflatedTopic) list.push(inflatedTopic);
    return list;
  }, [showRaw, showInflated, rawTopic, inflatedTopic]);

  // 建场景
  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    const mgr = new SceneManager(host);
    mgrRef.current = mgr;
    const ro = new ResizeObserver(() => mgr.resize());
    ro.observe(host);
    return () => {
      ro.disconnect();
      mgr.dispose();
      mgrRef.current = undefined;
    };
  }, []);

  // 订阅集合变化 → 增删图层
  useEffect(() => {
    const mgr = mgrRef.current;
    if (!mgr) return;
    const set = new Set(wanted);

    for (const [topic, unsub] of [...unsubRef.current]) {
      if (!set.has(topic)) {
        unsub();
        unsubRef.current.delete(topic);
        mgr.removeCloudLayer(topic);
      }
    }
    if (!conn) return;

    for (const topic of wanted) {
      if (unsubRef.current.has(topic)) continue;

      let frames = 0;
      let t0 = performance.now();
      let lastCount = 0;

      const unsub = conn.subscribe(topic, (msg) => {
        let parsed;
        try {
          parsed = parsePointCloud2(msg as PointCloud2Msg);
        } catch {
          return;
        }
        // 前端兜底降采样（正常应在 ROS 侧做）
        if (parsed.count > 60000) parsed = voxelDownsample(parsed, 0.1);
        lastCount = parsed.count;

        const bounds = mgrRef.current?.setCloudLayer(topic, parsed);
        if (!framedRef.current && bounds) {
          mgrRef.current?.frameCloud(bounds);
          framedRef.current = true;
        }

        frames++;
        const elapsed = (performance.now() - t0) / 1000;
        if (elapsed >= 1) {
          setStats((prev) => ({ ...prev, [topic]: { pts: lastCount, hz: frames / elapsed } }));
          frames = 0;
          t0 = performance.now();
        }
      });
      unsubRef.current.set(topic, unsub);
    }
  }, [conn, wanted]);

  useEffect(() => {
    if (wanted.length === 0) framedRef.current = false;
  }, [wanted.length]);

  useEffect(
    () => () => {
      unsubRef.current.forEach((f) => f());
      unsubRef.current.clear();
    },
    [],
  );

  // 无人机位姿
  useEffect(() => {
    if (!drone) return;
    mgrRef.current?.setDronePose(drone.x, drone.y, drone.z, drone.qx, drone.qy, drone.qz, drone.qw);
  }, [drone]);

  // 轨迹（绿）
  useEffect(() => {
    const mgr = mgrRef.current;
    if (!mgr) return;
    if (trajectory.length >= 6) mgr.setLineLayer("__traj", trajectory, "#48c46a");
    else mgr.clearLineLayer("__traj");
  }, [trajectory]);

  // 规划路径（橙）
  useEffect(() => {
    const mgr = mgrRef.current;
    if (!mgr) return;
    if (planPath.length >= 6) mgr.setLineLayer("__path", planPath, "#e0a030");
    else mgr.clearLineLayer("__path");
  }, [planPath]);

  // SUPER 期望轨迹（rviz 里的 ExpTraj）——这就是打点后立刻出现的那条。
  // 颜色取自 marker 自身，跟着 rviz 保持一致。
  useEffect(() => {
    const mgr = mgrRef.current;
    if (!mgr) return;
    if (expTraj.length >= 6) mgr.setLineLayer("__expTraj", expTraj, expTrajColor);
    else mgr.clearLineLayer("__expTraj");
  }, [expTraj, expTrajColor]);

  // 安全区棱柱（橙色线框 + 底面半透填充）
  useEffect(() => {
    const mgr = mgrRef.current;
    if (!mgr) return;
    mgr.setGeofence(
      geofence?.polygon ?? [],
      geofence?.zMin ?? 0,
      geofence?.zMax ?? 0,
      geofence?.enabled ?? false,
    );
  }, [geofence]);

  const totalPts = Object.values(stats).reduce((a, s) => a + s.pts, 0);
  const rawHz = stats[rawTopic]?.hz;
  const infHz = stats[inflatedTopic]?.hz;

  return (
    <div className="view3d">
      <div className="view3d-toolbar">
        <span className="lbl">点云</span>
        <div className="seg">
          <button
            className={showRaw ? "on" : ""}
            onClick={() => onShowRawChange(!showRaw)}
            disabled={!rawTopic}
            title={rawTopic || "未发现原生点云话题"}
          >
            原生{showRaw && rawHz ? ` ${rawHz.toFixed(0)}Hz` : ""}
          </button>
          <button
            className={showInflated ? "on" : ""}
            onClick={() => onShowInflatedChange(!showInflated)}
            disabled={!inflatedTopic}
            title={inflatedTopic || "未发现膨胀点云话题（SUPER 的 ROG Inflated）"}
          >
            膨胀{showInflated && infHz ? ` ${infHz.toFixed(0)}Hz` : ""}
          </button>
        </div>

        <span className="spacer" />

        <span className="axis-hint">
          <b style={{ color: "#48c46a" }}>轨迹</b>
          <b style={{ color: "#e0a030" }}>路径</b>
          {geofence?.enabled && <b style={{ color: "#ff9a3c" }}>安全区</b>}
          <b style={{ color: "#ff6060" }}>X 东</b>
          <b style={{ color: "#60e060" }}>Y 北</b>
          <b style={{ color: "#6aa8ff" }}>Z 上</b>
        </span>
        <div className="seg">
          <button onClick={() => mgrRef.current?.resetView()} title="从 -X 侧上方看">
            等距
          </button>
          <button onClick={() => mgrRef.current?.topView()} title="正俯视，与 2D 方向一致">
            俯视
          </button>
          <button
            className={drone ? "on" : ""}
            onClick={() => mgrRef.current?.setDroneVisible(true)}
            title="四旋翼模型"
          >
            模型
          </button>
        </div>
        <span className="muted mono">{totalPts.toLocaleString()} 点</span>
      </div>
      <div className="view3d-host" ref={hostRef} />
    </div>
  );
}
