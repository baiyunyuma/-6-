"""Embedded 3D arm preview: MuJoCo offscreen render + mouse TCP drag.

MuJoCo 的重活（场景更新、离屏渲染、预览 IK）全部在独立渲染线程里跑：
线程自持 preview 模型 / MjData / Renderer 和一份独立的 ArmKinematics
实例，与 GUI 线程、主窗口共享的运动学实例完全隔离。GUI 线程只做
鼠标交互、QPainter 叠加绘制和最新帧缓存；帧以 QImage 经排队信号回传
（QImage 允许在非 GUI 线程构造）。请求是 latest-wins：渲染线程忙时
新请求直接覆盖旧请求，不会积压。渲染线程崩溃只置错误文本，不拖垮
HMI 进程。Dragging the end-effector maps the cursor to a plane
perpendicular to the selected camera and solves pose IK in the render
thread.  Nothing here commands the real arm; the caller reads
``q_preview()`` when recording a waypoint.
"""
from __future__ import annotations

import math
import os
import threading
import time

import numpy as np
import mujoco
from PyQt5.QtCore import QObject, QPointF, QRect, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt5.QtWidgets import QWidget

CAM_W, CAM_H = 360, 270
RENDER_MAX_FPS = 80.0
#: 拖动中的 IK 节流周期（原 GUI 线程版本为 33 ms ≈ 30 Hz）
IK_DRAG_PERIOD_S = 0.033
TARGET_MIN = np.array([-0.9, -0.9, 0.03])
TARGET_MAX = np.array([0.9, 0.9, 1.3])
CAM_PRESETS = {
    "overview": (130.0, -18.0, 0.70),
    "front": (180.0, -8.0, 0.70),
    "side": (90.0, -8.0, 0.70),
    "top": (90.0, -80.0, 0.70),
}
#: 3D 预览专用的低模场景（每个零件约 3000 面）；缺失时回退原模型
PREVIEW_SCENE = os.path.expanduser(
    "~/zfy_arm/zfy_arm_control/model/scene_robot_arm_torque_preview.xml"
)


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def matrix_to_rpy(R) -> tuple:
    R = np.asarray(R, dtype=float)
    sy = max(-1.0, min(1.0, -R[2, 0]))
    pitch = math.asin(sy)
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    return roll, pitch, yaw


def _rot_axis(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def _point_segment_distance(p, a, b) -> float:
    p = np.asarray(p, dtype=float)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ab = b - a
    denom = float(np.dot(ab, ab))
    t = 0.0 if denom < 1e-12 else float(np.dot(p - a, ab)) / denom
    t = max(0.0, min(1.0, t))
    return float(np.linalg.norm(p - (a + t * ab)))


def _set_os_thread_name(name: str) -> None:
    """让 top -H / htop 能看到线程名（Linux 上限 15 字符）。"""
    try:
        import ctypes
        PR_SET_NAME = 15
        ctypes.CDLL("libc.so.6").prctl(PR_SET_NAME, name.encode()[:15], 0, 0, 0)
    except Exception:
        pass


# --------------------------------------------------------------------------- #
# 渲染线程
# --------------------------------------------------------------------------- #
class _Frame:
    """渲染线程 → GUI 的一帧结果。"""

    __slots__ = ("seq", "img", "view", "fovy", "q", "marker",
                 "did_ik", "err_mm", "err_deg", "error")

    def __init__(self):
        self.seq = -1
        self.img = None
        self.view = None
        self.fovy = 0.0
        self.q = None
        self.marker = None
        self.did_ik = False
        self.err_mm = 0.0
        self.err_deg = 0.0
        self.error = None


class _Request:
    """GUI → 渲染线程的请求；latest-wins，新请求直接覆盖未处理的旧请求。"""

    __slots__ = ("seq", "mode", "q", "target", "target_R",
                 "full_ik", "dragging", "cam")

    def __init__(self):
        self.seq = -1
        self.mode = "q"        # "q" = 直接显示 / "ik" = 先解 IK 再显示
        self.q = None
        self.target = None
        self.target_R = None
        self.full_ik = False
        self.dragging = False
        self.cam = (0.0, 0.0, 1.0, (0.0, 0.0, 0.0))


class _FrameEmitter(QObject):
    frame = pyqtSignal(object)


class _Context:
    """渲染线程自持的全部 MuJoCo 对象；GUI 线程从不触碰。"""

    __slots__ = ("kin", "model", "data", "renderer", "scene_option",
                 "qpos_idx", "cam", "target_mocap", "q", "fovy")


class _RenderWorker(threading.Thread):
    """独立渲染线程：IK + 场景更新 + 离屏渲染 + QImage 构造。

    MuJoCo 的 C 调用会释放 GIL，因此本线程与 GUI 线程真正并行。
    所有 MjModel/MjData/Renderer/GL 上下文都在本线程内创建、使用并销毁。
    """

    def __init__(self, kin_factory, emitter: "_FrameEmitter"):
        super().__init__(name="preview3d-render", daemon=True)
        self._kin_factory = kin_factory
        self._emitter = emitter
        self._cv = threading.Condition()
        self._req = None
        #: 不能叫 _stop：会覆盖 threading.Thread 的内部方法 _stop()
        self._stopping = False
        self._last_ik = 0.0
        self._last_render = 0.0

    # -------------------------------------------------------------- control
    def submit(self, req: _Request) -> None:
        with self._cv:
            self._req = req
            self._cv.notify()

    def stop(self) -> None:
        with self._cv:
            self._stopping = True
            self._cv.notify()

    # ----------------------------------------------------------------- loop
    def run(self):
        _set_os_thread_name("preview3d-rnd")
        try:
            ctx = self._init_context()
        except Exception as exc:
            self._emit_error(f"3D 渲染初始化失败: {exc}")
            return
        try:
            while True:
                with self._cv:
                    while self._req is None and not self._stopping:
                        self._cv.wait(0.5)
                    if self._stopping:
                        break
                    delay = self._pacing_delay(self._req)
                    if delay > 0.0:
                        # 等到限速到期；期间新请求可覆盖（自动合帧）
                        self._cv.wait(delay)
                        continue
                    req = self._req
                    self._req = None
                self._render(ctx, req)
        except Exception as exc:
            self._emit_error(f"3D 渲染失败: {exc}")
        finally:
            try:
                ctx.renderer.close()
            except Exception:
                pass

    def _pacing_delay(self, req: _Request) -> float:
        now = time.monotonic()
        delay = 0.0
        if req.mode == "ik" and req.dragging:
            delay = max(delay, IK_DRAG_PERIOD_S - (now - self._last_ik))
        if not req.full_ik:
            delay = max(delay, 1.0 / RENDER_MAX_FPS - (now - self._last_render))
        return max(0.0, delay)

    # -------------------------------------------------------------- context
    def _init_context(self) -> _Context:
        ctx = _Context()
        kin = self._kin_factory()
        model = kin.model
        if os.path.isfile(PREVIEW_SCENE):
            try:
                model = mujoco.MjModel.from_xml_path(PREVIEW_SCENE)
            except Exception:
                model = kin.model
        ctx.kin = kin
        ctx.model = model
        ctx.data = mujoco.MjData(model)
        qidx = []
        for jid in kin.joint_ids:
            name = mujoco.mj_id2name(
                kin.model, mujoco.mjtObj.mjOBJ_JOINT, int(jid)
            )
            pj = (
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if name
                else -1
            )
            if pj < 0:
                raise RuntimeError(f"preview model missing joint {name!r}")
            qidx.append(int(model.jnt_qposadr[pj]))
        ctx.qpos_idx = np.array(qidx, dtype=int)
        ctx.q = np.zeros(len(kin.joint_ids))
        # 预览不需要负载挂载点的蓝色球
        for body_name in ("payload", "payload_link3"):
            pid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, body_name
            )
            if pid < 0:
                continue
            for g in range(model.ngeom):
                if (
                    int(model.geom_bodyid[g]) == pid
                    and int(model.geom_type[g])
                    == int(mujoco.mjtGeom.mjGEOM_SPHERE)
                ):
                    model.geom_rgba[g, 3] = 0.0
        opt = mujoco.MjvOption()
        # 只画第 2 组：机械臂外观网格。地面、目标球、碰撞体都不进这一帧。
        opt.geomgroup[:] = 0
        opt.geomgroup[2] = 1
        opt.sitegroup[:] = 0
        opt.flags[mujoco.mjtVisFlag.mjVIS_STATIC] = True
        ctx.scene_option = opt
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [0.0, 0.0, 0.15]
        az, el, dist = CAM_PRESETS["overview"]
        cam.azimuth = az
        cam.elevation = el
        cam.distance = dist
        ctx.cam = cam
        ctx.target_mocap = -1
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
        if bid >= 0:
            ctx.target_mocap = int(model.body_mocapid[bid])
        ctx.fovy = math.radians(float(model.vis.global_.fovy))
        ctx.renderer = mujoco.Renderer(model, CAM_H, CAM_W)
        return ctx

    # --------------------------------------------------------------- render
    def _render(self, ctx: _Context, req: _Request) -> None:
        did_ik = False
        marker = None
        err_mm = 0.0
        err_deg = 0.0
        if req.mode == "ik":
            q, e_pos, e_rot, _ = ctx.kin.ik_pose(
                req.target, req.target_R, ctx.q,
                max_iter=60 if req.full_ik else 8,
            )
            ctx.q = q
            did_ik = True
            err_mm = float(e_pos) * 1000.0
            err_deg = math.degrees(float(e_rot))
            marker = ctx.kin.data.site_xpos[ctx.kin.tcp_site].copy()
            self._last_ik = time.monotonic()
        else:
            ctx.q = np.asarray(req.q, dtype=float).copy()

        cam = ctx.cam
        cam.azimuth, cam.elevation, cam.distance = req.cam[0], req.cam[1], req.cam[2]
        cam.lookat[:] = req.cam[3]
        data = ctx.data
        if ctx.target_mocap >= 0:
            data.mocap_pos[ctx.target_mocap] = req.target
        data.qpos[ctx.qpos_idx] = ctx.q
        mujoco.mj_kinematics(ctx.model, data)
        mujoco.mj_comPos(ctx.model, data)
        ctx.renderer.update_scene(
            data, camera=cam, scene_option=ctx.scene_option
        )
        scn = ctx.renderer.scene
        for flag in (
            mujoco.mjtRndFlag.mjRND_SHADOW,
            mujoco.mjtRndFlag.mjRND_REFLECTION,
            mujoco.mjtRndFlag.mjRND_SKYBOX,
            mujoco.mjtRndFlag.mjRND_HAZE,
            mujoco.mjtRndFlag.mjRND_FOG,
        ):
            scn.flags[int(flag)] = False
        img = ctx.renderer.render()
        # 渲染器实际使用的相机（固定相机接口在当前版本不可靠）
        sc = ctx.renderer.scene.camera[0]
        pos = np.array(sc.pos, dtype=float)
        fwd = np.array(sc.forward, dtype=float)
        up = np.array(sc.up, dtype=float)
        fwd /= np.linalg.norm(fwd)
        up /= np.linalg.norm(up)
        right = np.cross(fwd, up)
        right /= np.linalg.norm(right)
        f = 1.0 / math.tan(ctx.fovy * 0.5)
        h, w, _ = img.shape
        qimg = QImage(img.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        self._last_render = time.monotonic()

        fr = _Frame()
        fr.seq = req.seq
        fr.img = qimg
        fr.view = (pos, right, up, fwd, f, CAM_W / CAM_H)
        fr.fovy = ctx.fovy
        fr.q = ctx.q.copy()
        fr.marker = marker
        fr.did_ik = did_ik
        fr.err_mm = err_mm
        fr.err_deg = err_deg
        self._emitter.frame.emit(fr)

    def _emit_error(self, text: str) -> None:
        fr = _Frame()
        fr.error = text
        self._emitter.frame.emit(fr)


# --------------------------------------------------------------------------- #
# GUI 侧 widget：只留交互、叠加绘制和最新帧缓存
# --------------------------------------------------------------------------- #
class Preview3D(QWidget):
    status = pyqtSignal(str)
    poseChanged = pyqtSignal(float, float, float)

    def __init__(self, kin, parent=None, kin_factory=None):
        super().__init__(parent)
        self.kin = kin
        self.n = len(kin.joint_ids)
        self._active_mask = np.ones(self.n, dtype=bool)
        if kin_factory is None:
            scene = getattr(kin, "scene", None)
            if not scene:
                raise ValueError(
                    "Preview3D 需要 kin_factory，或让 kin 携带 scene 路径，"
                    "供渲染线程自建独立的运动学实例"
                )
            kin_factory = lambda: type(kin)(scene)
        self.q = np.zeros(self.n)
        self.q_real = self.q.copy()
        self.previewing = False
        self.dragging = False
        self._drag_depth = 0.0
        self._drag_kind = None
        self._last_mouse = None
        self._drag_origin = None
        self._drag_target0 = None
        self._ring_axis = None
        self._ring_R0 = None
        self._ring_center = None
        self._ring_start = 0.0
        self.target, self.target_R = self.kin.tcp_pose(self.q)
        self._marker_tcp = self.target.copy()
        self._ik_due = False
        #: 提交序号；reset/跟随切换会抬高下限，丢弃此前在飞的过期 IK 帧
        self._seq = 0
        self._frame_floor = 0
        self._img = None
        self._img_rect = QRect()
        self._dirty = True
        self._err_mm = 0.0
        self._err_deg = 0.0
        self._camera_name = "overview"
        az, el, dist = CAM_PRESETS["overview"]
        self._cam_az = float(az)
        self._cam_el = float(el)
        self._cam_dist = float(dist)
        self._cam_lookat = np.array([0.0, 0.0, 0.15])
        self._view = None
        self._fovy = math.radians(float(kin.model.vis.global_.fovy))
        self._error = None
        self._worker_dead = False
        self._emitter = _FrameEmitter(self)
        self._emitter.frame.connect(self._on_frame)
        # ZFY_NO_3D=1: skip the MuJoCo/OpenGL render thread.  That thread is a
        # native crash source under WSLg; joint control, gamepad IK, recording
        # and replay do not need it.
        self._no_3d = os.environ.get("ZFY_NO_3D", "").strip().lower() in (
            "1", "true", "yes", "on"
        )
        if self._no_3d:
            self._worker = None
            self._worker_dead = True
            self._error = "3D 预览已禁用（ZFY_NO_3D=1）"
        else:
            self._worker = _RenderWorker(kin_factory, self._emitter)
            self._worker.start()
        self.setMinimumSize(420, 300)
        self.setMouseTracking(True)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        if not self._no_3d:
            self._timer.start(max(1, int(1000.0 / RENDER_MAX_FPS)))

    # ------------------------------------------------------------------ state
    def q_preview(self) -> np.ndarray:
        return self.q.copy()

    def is_previewing(self) -> bool:
        return self.previewing

    def ik_error(self):
        return float(self._err_mm), float(self._err_deg)

    def status_text(self) -> str:
        if self.previewing:
            return (
                f"预览位姿 · 位置 {self._err_mm:.1f} mm · "
                f"姿态 {self._err_deg:.1f}°"
            )
        return "实机快照"

    def set_active_mask(self, mask) -> None:
        m = np.asarray(mask, dtype=bool)
        if m.shape != (self.n,):
            return
        self._active_mask = m.copy()
        self._dirty = True

    def set_real_q(self, q) -> None:
        q = np.asarray(q, dtype=float).copy()
        merged = self.q.copy()
        merged[self._active_mask] = q[self._active_mask]
        self.q_real = merged.copy()
        if self.previewing:
            return
        if np.max(np.abs(self.q_real - self.q)) < 1e-6:
            return
        self.q = self.q_real.copy()
        self.target, self.target_R = self.kin.tcp_pose(self.q)
        self._marker_tcp = self.target.copy()
        self._ik_due = False
        self._bump_seq()
        self._dirty = True

    def show_motion(self, q) -> None:
        """跟随关节指令插值，不把画面当成鼠标预览。"""
        if self.dragging or self.previewing:
            return
        q = np.asarray(q, dtype=float).copy()
        if q.shape != self.q.shape:
            return
        merged = self.q.copy()
        merged[self._active_mask] = q[self._active_mask]
        if np.max(np.abs(merged - self.q)) < 5e-4:
            return
        self.q = merged
        self.q_real = merged.copy()
        self._ik_due = False
        self._dirty = True
        self.status.emit("指令运动")

    def finish_motion(self, q) -> None:
        if self.dragging or self.previewing:
            return
        q = np.asarray(q, dtype=float).copy()
        merged = self.q.copy()
        merged[self._active_mask] = q[self._active_mask]
        self.q = merged
        self.q_real = merged.copy()
        self.target, self.target_R = self.kin.tcp_pose(self.q)
        self._marker_tcp = self.target.copy()
        self._ik_due = False
        self._bump_seq()
        self._dirty = True
        self.status.emit(self.status_text())
        self.poseChanged.emit(*self.target_rpy_deg())

    def reset_to_real(self, q=None) -> None:
        if q is not None:
            q = np.asarray(q, dtype=float).copy()
            merged = self.q.copy()
            merged[self._active_mask] = q[self._active_mask]
            self.q_real = merged.copy()
        else:
            merged = self.q.copy()
            merged[self._active_mask] = self.q_real[self._active_mask]
            self.q_real = merged.copy()
        self.q = self.q_real.copy()
        self.target, self.target_R = self.kin.tcp_pose(self.q)
        self._marker_tcp = self.target.copy()
        self._err_mm = 0.0
        self._err_deg = 0.0
        self.previewing = False
        self.dragging = False
        self._drag_kind = None
        self._ik_due = False
        self._bump_seq()
        self._dirty = True
        self.status.emit(self.status_text())
        self.poseChanged.emit(*self.target_rpy_deg())

    def _bump_seq(self) -> None:
        self._seq += 1
        self._frame_floor = self._seq

    def target_rpy_deg(self):
        roll, pitch, yaw = matrix_to_rpy(self.target_R)
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def set_target_rpy(self, roll: float, pitch: float, yaw: float) -> None:
        self.target_R = rpy_to_matrix(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )
        self._apply_pose()

    def _apply_pose(self) -> None:
        # 拖动时不要在每次鼠标事件里跑完整逆解；渲染线程按 30 Hz 合并
        self._ik_due = True
        self.previewing = True

    def _ring_image_points(self, i: int, segments: int = 24):
        j, k = ((1, 2), (2, 0), (0, 1))[i]
        pts = []
        for s in range(segments + 1):
            th = 2.0 * math.pi * s / segments
            p3 = self.target + (
                self.target_R[:, j] * math.cos(th)
                + self.target_R[:, k] * math.sin(th)
            ) * 0.06
            uv = self._project(p3)
            if uv is None:
                return None
            pts.append(uv)
        return pts

    def _hit_ring(self, img_xy):
        best = None
        best_d = 14.0
        for i in range(3):
            pts = self._ring_image_points(i)
            if not pts:
                continue
            for a, b in zip(pts[:-1], pts[1:]):
                d = _point_segment_distance(img_xy, a, b)
                if d < best_d:
                    best_d = d
                    best = i
        return best

    def _hit_center(self, img_xy) -> bool:
        uv = self._project(self.target)
        if uv is None:
            return False
        return math.hypot(img_xy[0] - uv[0], img_xy[1] - uv[1]) <= 28.0

    def set_camera(self, name: str) -> None:
        preset = CAM_PRESETS.get(name)
        if preset is None or name == self._camera_name:
            return
        az, el, dist = preset
        self._cam_az = float(az)
        self._cam_el = float(el)
        self._cam_dist = float(dist)
        self._cam_lookat = np.array([0.0, 0.0, 0.15])
        self._camera_name = name
        self._dirty = True

    def _orbit(self, dx: float, dy: float) -> None:
        # 鼠标方向 = 画面移动方向（向右拖，视角向右；向上拖，视角向上）
        self._cam_az = (self._cam_az - dx * 0.4) % 360.0
        self._cam_el = max(-89.0, min(89.0, self._cam_el - dy * 0.4))
        self._camera_name = "free"
        self._dirty = True

    def _pan(self, dx: float, dy: float) -> None:
        basis = self._cam_basis()
        if basis is None:
            return
        _pos, right, up, _forward, _f, _aspect = basis
        mpp = 2.0 * float(self._cam_dist) * math.tan(self._fovy * 0.5) / CAM_H
        self._cam_lookat = self._cam_lookat + (-right * dx + up * dy) * mpp
        self._camera_name = "free"
        self._dirty = True

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta == 0:
            return
        self._cam_dist = max(
            0.45, min(1.8, self._cam_dist * (0.9 if delta > 0 else 1.1))
        )
        self._dirty = True

    # ------------------------------------------------------------- rendering
    def _tcp(self, q) -> np.ndarray:
        return self.kin.fk(q)

    def _tick(self) -> None:
        """GUI 侧的轻量节拍（80 Hz）：只打包状态提交给渲染线程。"""
        if self._worker is None or self._worker_dead:
            return
        if not (self._dirty or self._ik_due):
            return
        # 3D 页不在前台 / 窗口最小化时不要渲染，保留 dirty 等重新显示
        if not self.isVisible() or self.window().isMinimized():
            return
        req = _Request()
        req.seq = self._seq
        req.target = self.target.copy()
        req.target_R = self.target_R.copy()
        req.dragging = bool(self.dragging)
        req.cam = (
            self._cam_az, self._cam_el, self._cam_dist,
            (float(self._cam_lookat[0]), float(self._cam_lookat[1]),
             float(self._cam_lookat[2])),
        )
        if self._ik_due:
            req.mode = "ik"
            req.full_ik = not self.dragging
            self._ik_due = False
        else:
            req.mode = "q"
            req.q = self.q.copy()
        self._dirty = False
        self._worker.submit(req)

    def _on_frame(self, f: _Frame) -> None:
        if f.error is not None:
            self._worker_dead = True
            self._error = f.error
            self.update()
            return
        if f.seq < self._frame_floor:
            return  # reset/切回实机跟随之前的过期帧，丢弃
        self._img = f.img
        self._view = f.view
        self._fovy = f.fovy
        if f.did_ik:
            # 只有 IK 帧才回写 q：显示帧的 q 本来就来自 GUI 侧，
            # 回写反而会让快速跟随（show_motion）出现一帧回退。
            if f.q is not None:
                self.q = f.q
            self._err_mm = f.err_mm
            self._err_deg = f.err_deg
            if f.marker is not None:
                self._marker_tcp = f.marker
            self.previewing = True
            self.status.emit(self.status_text())
        self.update()

    # ----------------------------------------------------------- camera math
    def _cam_basis(self):
        return self._view

    def _project(self, p):
        basis = self._cam_basis()
        if basis is None:
            return None
        pos, right, up, forward, f, aspect = basis
        rel = np.asarray(p, dtype=float) - pos
        z = float(np.dot(rel, forward))
        if z <= 1e-5:
            return None
        x = float(np.dot(rel, right)) * f / (aspect * z)
        y = float(np.dot(rel, up)) * f / z
        return (x * 0.5 + 0.5) * CAM_W, (0.5 - y * 0.5) * CAM_H

    def _unproject(self, sx, sy, depth):
        basis = self._cam_basis()
        if basis is None:
            return None
        pos, right, up, forward, f, aspect = basis
        x = sx / CAM_W * 2.0 - 1.0
        y = 1.0 - sy / CAM_H * 2.0
        direction = forward + right * (x * aspect / f) + up * (y / f)
        direction /= np.linalg.norm(direction)
        denom = float(np.dot(direction, forward))
        if abs(denom) < 1e-6:
            return None
        return pos + direction * (depth / denom)

    def _image_to_widget(self, x, y):
        rect = self._img_rect
        if rect.isNull():
            return None
        return QPointF(
            rect.x() + x / CAM_W * rect.width(),
            rect.y() + y / CAM_H * rect.height(),
        )

    def _widget_to_image(self, x, y):
        rect = self._img_rect
        if rect.isNull() or not rect.contains(int(x), int(y)):
            return None
        return (
            (x - rect.x()) / rect.width() * CAM_W,
            (y - rect.y()) / rect.height() * CAM_H,
        )

    # --------------------------------------------------------------- painting
    def paintEvent(self, _event):
        p = QPainter(self)
        try:
            self._paint(p)
        except Exception as exc:
            self._error = f"3D 绘制失败: {exc}"
        finally:
            p.end()

    def _paint(self, p: QPainter):
        p.fillRect(self.rect(), QColor(16, 18, 22))
        if self._img is None:
            p.setPen(QColor(220, 140, 140))
            p.drawText(self.rect(), Qt.AlignCenter, self._error or "3D 渲染初始化中…")
            return
        iw, ih = self._img.width(), self._img.height()
        scale = min(self.width() / iw, self.height() / ih)
        w, h = int(iw * scale), int(ih * scale)
        self._img_rect = QRect(
            (self.width() - w) // 2, (self.height() - h) // 2, w, h
        )
        p.drawImage(self._img_rect, self._img)

        tcp = self._marker_tcp
        proj = self._project(tcp)
        if proj is not None:
            pt = self._image_to_widget(*proj)
            if pt is not None:
                p.setPen(QPen(QColor(70, 220, 140), 2))
                p.drawEllipse(pt, 5, 5)
        proj_t = self._project(self.target)
        if proj_t is not None:
            pt = self._image_to_widget(*proj_t)
            if pt is not None:
                p.setPen(QPen(QColor(255, 150, 40), 1, Qt.DashLine))
                p.drawEllipse(pt, 14, 14)
        # 三个旋转环（X 红 / Y 绿 / Z 蓝）
        for i, color in enumerate((
            QColor(255, 80, 80, 220),
            QColor(80, 220, 80, 220),
            QColor(90, 150, 255, 220),
        )):
            pts = self._ring_image_points(i)
            if not pts:
                continue
            poly = [self._image_to_widget(x, y) for x, y in pts]
            if any(pt is None for pt in poly):
                continue
            p.setPen(QPen(color, 2))
            for a, b in zip(poly[:-1], poly[1:]):
                p.drawLine(a, b)

        font = QFont(p.font())
        font.setPointSize(9)
        p.setFont(font)
        bad = self.previewing and (self._err_mm > 5.0 or self._err_deg > 5.0)
        p.setPen(QColor(240, 160, 90) if bad else QColor(225, 230, 240))
        text = self.status_text() + f" · {self._camera_name}"
        tw = p.fontMetrics().horizontalAdvance(text) + 14
        p.fillRect(8, 8, tw, 22, QColor(0, 0, 0, 150))
        p.drawText(15, 24, text)
        p.setPen(QColor(160, 170, 185))
        p.drawText(
            12, self.height() - 12,
            "左键拖中心=平移，拖彩色环=绕轴旋转；右键拖=转视角；中键=平移视角；滚轮=缩放",
        )

    # ------------------------------------------------------------------ mouse
    def mousePressEvent(self, event):
        if self._img is None:
            return super().mousePressEvent(event)
        img = self._widget_to_image(event.x(), event.y())
        if img is None:
            return super().mousePressEvent(event)
        if event.button() == Qt.MiddleButton:
            self._start_drag("pan", event)
            return
        if event.button() == Qt.RightButton:
            kind = "pan" if event.modifiers() & Qt.ShiftModifier else "orbit"
            self._start_drag(kind, event)
            return
        if event.button() != Qt.LeftButton:
            return super().mousePressEvent(event)
        if self._hit_center(img):
            basis = self._cam_basis()
            if basis is None:
                return
            pos, _right, _up, forward, _f, _aspect = basis
            self._drag_depth = float(np.dot(self.target - pos, forward))
            origin = self._unproject(img[0], img[1], self._drag_depth)
            if origin is None:
                return
            self._drag_origin = origin
            self._drag_target0 = self.target.copy()
            self.dragging = True
            self._drag_kind = "translate"
            event.accept()
            return
        ring = self._hit_ring(img)
        if ring is not None:
            uv = self._project(self.target)
            if uv is None:
                return
            cw = self._image_to_widget(*uv)
            if cw is None:
                return
            self._ring_axis = self.target_R[:, ring].copy()
            self._ring_R0 = self.target_R.copy()
            self._ring_center = cw
            self._ring_start = math.atan2(
                event.y() - cw.y(), event.x() - cw.x()
            )
            self.dragging = True
            self._drag_kind = "ring_rotate"
            event.accept()
            return
        # 左键拖空白 = 转视角
        self._start_drag("orbit", event)

    def _start_drag(self, kind: str, event) -> None:
        self.dragging = True
        self._drag_kind = kind
        self._last_mouse = (event.x(), event.y())
        event.accept()

    def mouseMoveEvent(self, event):
        if not self.dragging:
            return
        if self._drag_kind == "ring_rotate":
            if self._ring_center is None:
                return
            c = self._ring_center
            ang = math.atan2(event.y() - c.y(), event.x() - c.x())
            delta = (ang - self._ring_start + math.pi) % (2.0 * math.pi) - math.pi
            basis = self._cam_basis()
            sign = (
                -1.0
                if basis is not None
                and float(np.dot(self._ring_axis, basis[3])) < 0.0
                else 1.0
            )
            R = _rot_axis(self._ring_axis, sign * delta) @ self._ring_R0
            U, _S, Vt = np.linalg.svd(R)
            self.target_R = U @ Vt
            self._apply_pose()
            self.poseChanged.emit(*self.target_rpy_deg())
            return
        if self._drag_kind in ("orbit", "pan"):
            if self._last_mouse is None:
                self._last_mouse = (event.x(), event.y())
                return
            lx, ly = self._last_mouse
            dx = event.x() - lx
            dy = event.y() - ly
            self._last_mouse = (event.x(), event.y())
            if abs(dx) + abs(dy) < 1:
                return
            if self._drag_kind == "orbit":
                self._orbit(dx, dy)
            else:
                self._pan(dx, dy)
            return
        img = self._widget_to_image(event.x(), event.y())
        if img is None or self._drag_origin is None:
            return
        p = self._unproject(img[0], img[1], self._drag_depth)
        if p is None:
            return
        self.target = np.clip(
            self._drag_target0 + (p - self._drag_origin), TARGET_MIN, TARGET_MAX
        )
        self._apply_pose()

    def mouseReleaseEvent(self, event):
        if event.button() in (Qt.LeftButton, Qt.RightButton, Qt.MiddleButton):
            was_pose = self._drag_kind in ("translate", "ring_rotate")
            self.dragging = False
            self._drag_kind = None
            if was_pose:
                # 松手后下一个 _tick 提交 full IK（dragging 已为 False）
                self._ik_due = True
                self._dirty = True
            event.accept()

    def closeEvent(self, event):
        self._timer.stop()
        worker = self._worker
        self._worker = None
        if worker is not None:
            worker.stop()
            worker.join(timeout=1.5)
        super().closeEvent(event)
