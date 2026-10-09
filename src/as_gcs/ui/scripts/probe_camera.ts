/**
 * 验证 3D 视角的相机朝向（纯计算，不开浏览器）。
 *
 * 要确认两件事：
 *   1. 「等距」和「俯视」的 camera.up 都是 three 默认的 (0,1,0)
 *      —— OrbitControls 拿 camera.up 当极轴，up 一致 = 鼠标拖拽的旋转轴一致。
 *   2. 俯视时屏幕上方仍然是 ROS 的 +X（东）、右方是 ROS 的 -Y（南），
 *      也就是和 2D 地图方向一致。
 *
 * 用法：npx vite-node scripts/probe_camera.ts
 */
import * as THREE from "three";

const FOV = 60;
const ASPECT = 1.6;

/** 复刻 SceneManager 的初始等距视角 */
function isoView() {
  const cam = new THREE.PerspectiveCamera(FOV, ASPECT, 0.05, 2000);
  cam.up.set(0, 1, 0);
  const target = new THREE.Vector3(0, 0, 0);
  cam.position.set(-9, 4.5, 1.2);
  cam.lookAt(target);
  return cam;
}

/** 复刻改动后的 topView() */
function topViewNew() {
  const cam = new THREE.PerspectiveCamera(FOV, ASPECT, 0.05, 2000);
  cam.up.set(0, 1, 0);
  const t = new THREE.Vector3(0, 0, 0);
  const d = 14;
  cam.position.set(t.x - d * 0.14, t.y + d, t.z);
  cam.lookAt(t);
  return cam;
}

/** 复刻改动前的 topView()（把 up 改成了 three +X） */
function topViewOld() {
  const cam = new THREE.PerspectiveCamera(FOV, ASPECT, 0.05, 2000);
  const t = new THREE.Vector3(0, 0, 0);
  cam.up.set(1, 0, 0);
  cam.position.set(t.x, t.y + 18, t.z);
  cam.lookAt(t);
  return cam;
}

const toRos = (v: THREE.Vector3) => ({ x: v.x, y: -v.z, z: v.y });
const label = (v: THREE.Vector3) => {
  const r = toRos(v);
  const near = (a: number, b: number) => Math.abs(a - b) < 0.25;
  if (near(r.x, 1) && near(r.y, 0) && near(r.z, 0)) return "+X 东";
  if (near(r.x, -1)) return "-X 西";
  if (near(r.y, 1)) return "+Y 北";
  if (near(r.y, -1)) return "-Y 南";
  if (near(r.z, 1)) return "+Z 上";
  return "(斜向)";
};

function report(name: string, cam: THREE.PerspectiveCamera) {
  cam.updateMatrixWorld();
  const rot = new THREE.Matrix4().extractRotation(cam.matrixWorld);
  const up = new THREE.Vector3(0, 1, 0).applyMatrix4(rot);
  const right = new THREE.Vector3(1, 0, 0).applyMatrix4(rot);
  console.log(`\n${name}`);
  console.log(`  camera.up          = (${cam.up.x.toFixed(2)}, ${cam.up.y.toFixed(2)}, ${cam.up.z.toFixed(2)})`);
  console.log(`  相机位置           = (${cam.position.x.toFixed(2)}, ${cam.position.y.toFixed(2)}, ${cam.position.z.toFixed(2)})`);
  console.log(`  屏幕上方 → ROS     = ${label(up)}   [${toRos(up).x.toFixed(3)}, ${toRos(up).y.toFixed(3)}, ${toRos(up).z.toFixed(3)}]`);
  console.log(`  屏幕右方 → ROS     = ${label(right)}   [${toRos(right).x.toFixed(3)}, ${toRos(right).y.toFixed(3)}, ${toRos(right).z.toFixed(3)}]`);
}

const iso = isoView();
const topNew = topViewNew();
const topOld = topViewOld();

report("【等距】(resetView)", iso);
report("【俯视】改动后 (topView)", topNew);
report("【俯视】改动前 (topView)", topOld);

console.log("\n═══ 结论 ═══");
const sameUp =
  iso.up.x === topNew.up.x && iso.up.y === topNew.up.y && iso.up.z === topNew.up.z;
console.log(`  等距与俯视的 camera.up 是否一致: ${sameUp ? "✅ 一致（鼠标旋转轴相同）" : "❌ 不一致"}`);
const oldSameUp =
  iso.up.x === topOld.up.x && iso.up.y === topOld.up.y && iso.up.z === topOld.up.z;
console.log(`  （改动前）等距与俯视的 up 是否一致: ${oldSameUp ? "一致" : "❌ 不一致 —— 这就是“俯视手感不一样”的根源"}`);
