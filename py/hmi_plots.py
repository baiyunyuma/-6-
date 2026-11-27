"""达妙电机四联反馈曲线（PyQt5 + pyqtgraph）。

布局与行为对齐 DM_DJI_TOOL_V1_src 的达妙曲线页：
  * 位置 / 速度 / 力矩 / 前馈力矩 四个子图；
  * 每台电机一条实线（按 CAN ID 稳定配色），可叠加「期望值」虚线；
  * 峰值保持降采样（真正的挑点在 telemetry.TelemetryBuffer 里做）；
  * 十字线读数、暂停滚动、网格开关、重置视图、PNG 导出。

本模块只负责画，不碰 SDK、不碰 worker。
"""
from __future__ import annotations

import time

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)

#: 每台电机一个固定颜色（按 CAN ID 排序取）。四个子图用同一套颜色，
#: 这样「红色 = J1、绿色 = J2」在所有子图里都成立，6 轴时比明暗色好认。
MOTOR_COLORS = (
    (229, 57, 53),     # 红
    (67, 160, 71),     # 绿
    (30, 136, 229),    # 蓝
    (255, 152, 0),     # 橙
    (156, 39, 176),    # 紫
    (0, 188, 212),     # 青
    (255, 193, 7),     # 琥珀
    (141, 110, 99),    # 棕
)

#: 曲线渲染节拍（毫秒）。渲染频率与采样频率彻底解耦。
RENDER_INTERVAL_MS = 33
RENDER_INTERVAL_MAX_MS = 120
RENDER_ADAPT_COOLDOWN_S = 0.7
#: 每条曲线按「绘制点/像素」定量，避免电机越多画得越慢
CURVE_POINTS_PER_PX = 1.0
CURVE_POINTS_MIN = 250
CURVE_POINTS_MAX = 3000


def _apply_curve_perf(item):
    try:
        item.setClipToView(True)
        item.setDownsampling(auto=True, mode="peak")
        item.opts["autoDownsampleFactor"] = 1.0
    except Exception:
        pass


class PlotPanel(QWidget):
    """四个子图 + 工具栏 + 十字线。"""

    record_toggled = pyqtSignal(bool)
    export_csv_requested = pyqtSignal()
    export_png_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._x_window = 10.0
        self.curves = {}            # can_id -> {"pos"/"vel"/"tau"/"ff_tau"/"cmd_*": PlotDataItem}
        self.buffers = {}           # can_id -> {"t": ndarray, ...} 最近一帧（十字线读数用）
        self._rendered_mark = {}    # can_id -> 上次渲染的最新采样时刻
        self._series_index = {}
        self._plot_t0 = None
        self._build_ui()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 10, 6, 4)

        top = QHBoxLayout()
        self.chk_pause_scroll = QCheckBox("暂停滚动（可缩放）")
        top.addWidget(self.chk_pause_scroll)

        self.chk_show_cmd = QCheckBox("显示期望值")
        self.chk_show_cmd.setChecked(False)
        self.chk_show_cmd.toggled.connect(self._on_toggle_cmd)
        top.addWidget(self.chk_show_cmd)

        self.chk_grid = QCheckBox("显示网格")
        self.chk_grid.setChecked(False)
        self.chk_grid.setToolTip(
            "网格横线横跨整个绘图区，软件光栅化下相当贵。\n"
            "调试觉得卡就保持关闭，需要读数参照时再临时打开。")
        self.chk_grid.toggled.connect(self._apply_grid)
        top.addWidget(self.chk_grid)

        self.chk_record = QCheckBox("记录数据")
        self.chk_record.setChecked(True)
        self.chk_record.setToolTip("关闭后停止采样（省 CPU），已采集的数据保留")
        self.chk_record.toggled.connect(self.record_toggled.emit)
        top.addWidget(self.chk_record)

        btn_reset = QPushButton("重置视图")
        btn_reset.clicked.connect(self.reset_view)
        top.addWidget(btn_reset)
        btn_csv = QPushButton("导出 CSV")
        btn_csv.clicked.connect(self.export_csv_requested.emit)
        top.addWidget(btn_csv)
        btn_png = QPushButton("导出 PNG")
        btn_png.clicked.connect(self.export_png_requested.emit)
        top.addWidget(btn_png)
        top.addStretch(1)
        lay.addLayout(top)

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setMinimumHeight(150)
        self.plot_pos = self.plot_widget.addPlot(row=0, col=0, title="位置 (rad)")
        self.plot_vel = self.plot_widget.addPlot(row=1, col=0, title="速度 (rad/s)")
        self.plot_tau = self.plot_widget.addPlot(row=2, col=0, title="力矩 (N·m)")
        self.plot_ff = self.plot_widget.addPlot(row=3, col=0, title="前馈力矩 (N·m)")
        for p in self._plots():
            p.showGrid(x=False, y=False)
            p.enableAutoRange(axis="y")
            p.enableAutoRange(axis="x", enable=False)
            p.setClipToView(True)
            p.setDownsampling(auto=True, mode="peak")
            p.setXRange(0, 10, padding=0)
            p.getViewBox().sigXRangeChanged.connect(self._on_x_range_changed)
            for side in ("left", "bottom"):
                # 关掉 SI 前缀自动缩放：否则数值上千会被改写成 "(x1000)"，
                # 看着像量纲被换了，其实数据没动。
                p.getAxis(side).enableAutoSIPrefix(False)
            p.getAxis("left").setWidth(42)     # 留够负号和小数位

        # 只在「位置」子图放一个图例：四个子图的电机配色相同，每个都放图例
        # 会让 6 台电机时图例占掉大半个绘图区。期望值虚线不进图例。
        lg = self.plot_pos.addLegend(offset=(8, 8))
        lg.setLabelTextSize("9pt")
        lg.setBrush(pg.mkBrush(0, 0, 0, 150))
        lg.setPen(pg.mkPen(130, 130, 130, 160))
        lay.addWidget(self.plot_widget, 1)

        self.lbl_crosshair = QLabel("")
        self.lbl_crosshair.setStyleSheet(
            "color: #ccc; font-family: monospace; font-size: 11px;")
        self.lbl_crosshair.setWordWrap(True)
        lay.addWidget(self.lbl_crosshair)

        self._setup_crosshair()

    def _plots(self):
        return (self.plot_pos, self.plot_vel, self.plot_tau, self.plot_ff)

    def _apply_grid(self, on):
        for p in self._plots():
            try:
                p.showGrid(x=bool(on), y=bool(on), alpha=0.3)
            except Exception:
                pass

    # -------------------------------------------------------------- 曲线归属
    @staticmethod
    def _pen(_signal, idx):
        return pg.mkPen(color=MOTOR_COLORS[idx % len(MOTOR_COLORS)], width=2)

    @staticmethod
    def _pen_cmd(_signal, idx):
        return pg.mkPen(color=MOTOR_COLORS[idx % len(MOTOR_COLORS)],
                        width=1, style=Qt.DashLine)

    def set_motors(self, can_ids):
        """按 CAN ID 排序分配稳定的明暗索引，然后重建曲线集合。"""
        self._series_index = {int(cid): i for i, cid in enumerate(sorted(can_ids))}
        # 重新连接时电机集合可能变了；旧曲线全部移除，按需惰性重建，
        # 否则图例会越连越多。
        for cid, cur in list(self.curves.items()):
            for item in cur.values():
                for p in self._plots():
                    try:
                        p.removeItem(item)
                    except Exception:
                        pass
        self.curves.clear()
        self.clear_data()

    def _add_plot_series(self, can_id):
        idx = self._series_index.get(can_id, len(self.curves))
        label = "0x%X" % can_id
        show_cmd = self.chk_show_cmd.isChecked()
        self.curves[can_id] = {
            "pos": self.plot_pos.plot(pen=self._pen("pos", idx), name=label),
            "vel": self.plot_vel.plot(pen=self._pen("vel", idx), name=label),
            "tau": self.plot_tau.plot(pen=self._pen("tau", idx), name=label),
            "ff_tau": self.plot_ff.plot(pen=self._pen("ff_tau", idx), name=label),
            # 期望值虚线**不进图例**：它们默认隐藏，若进图例会把条目数翻倍，
            # 6 台电机时图例直接盖住半个绘图区。同色虚线本身已经足够区分。
            "cmd_pos": self.plot_pos.plot(pen=self._pen_cmd("pos", idx)),
            "cmd_vel": self.plot_vel.plot(pen=self._pen_cmd("vel", idx)),
            "cmd_tau": self.plot_tau.plot(pen=self._pen_cmd("tau", idx)),
        }
        for key in ("cmd_pos", "cmd_vel", "cmd_tau"):
            self.curves[can_id][key].setVisible(show_cmd)
        for item in self.curves[can_id].values():
            _apply_curve_perf(item)
        # 电机多时图例分两列，避免又高又窄
        try:
            self.plot_pos.legend.setColumnCount(2 if len(self.curves) > 4 else 1)
        except Exception:
            pass

    def _on_toggle_cmd(self):
        show = self.chk_show_cmd.isChecked()
        for cur in self.curves.values():
            for key in ("cmd_pos", "cmd_vel", "cmd_tau"):
                cur[key].setVisible(show)
        # 立刻补画一次，不用等下一个渲染节拍
        self._rendered_mark.clear()

    def clear_data(self):
        for cur in self.curves.values():
            for item in cur.values():
                item.setData([], [])
        self.buffers.clear()
        self._rendered_mark.clear()
        self._plot_t0 = None
        self._x_window = 10.0
        for p in self._plots():
            p.setXRange(0, 10, padding=0)

    # ------------------------------------------------------------------ 渲染
    def render(self, samples):
        if not samples:
            if not self.chk_pause_scroll.isChecked() and self._plot_t0 is not None:
                self._scroll(time.monotonic() - self._plot_t0)
            return

        show_cmd = self.chk_show_cmd.isChecked()
        t_max = 0.0
        for can_id, (t_arr, chans) in samples.items():
            n = len(t_arr)
            if n == 0:
                continue
            if self._plot_t0 is None:
                # 时间基准只定一次 = 第一帧采样的时刻，X 轴读数就是从 0 起的秒数
                self._plot_t0 = float(t_arr[0])
            xs = t_arr - self._plot_t0
            t_max = max(t_max, float(xs[-1]))

            if can_id not in self.curves:
                self._add_plot_series(can_id)
            # 只给十字线读数/导出保留最近一帧，不复制整段历史
            self.buffers[can_id] = {"t": xs, **chans}

            # 判据必须是「最新采样时刻」。降采样后的点数是个常数，
            # 拿点数判断会让曲线永久冻结。
            mark = float(t_arr[-1])
            if self._rendered_mark.get(can_id) == mark:
                continue
            self._rendered_mark[can_id] = mark

            cur = self.curves[can_id]
            cur["pos"].setData(xs, chans["pos"])
            cur["vel"].setData(xs, chans["vel"])
            cur["tau"].setData(xs, chans["tau"])
            cur["ff_tau"].setData(xs, chans["ff_tau"])
            if show_cmd:
                cur["cmd_pos"].setData(xs, chans["cmd_q"])
                cur["cmd_vel"].setData(xs, chans["cmd_dq"])
                cur["cmd_tau"].setData(xs, chans["cmd_tau"])

        if not self.chk_pause_scroll.isChecked() and t_max > 0.0:
            self._scroll(t_max)

    def _scroll(self, t):
        lo, hi = (t - self._x_window, t) if t > self._x_window else (0.0, self._x_window)
        for p in self._plots():
            cur = p.getViewBox().viewRange()[0]
            if abs(cur[0] - lo) > 1e-6 or abs(cur[1] - hi) > 1e-6:
                p.setXRange(lo, hi, padding=0)

    def _on_x_range_changed(self, _vb, rng):
        w = rng[1] - rng[0]
        if w > 0.01:
            self._x_window = w

    def reset_view(self):
        self._x_window = 10.0
        for p in self._plots():
            p.enableAutoRange(axis="y")
        t = 0.0
        for b in self.buffers.values():
            if len(b["t"]):
                t = max(t, float(b["t"][-1]))
        self._scroll(t)

    # --------------------------------------------------------------- 十字线
    def _setup_crosshair(self):
        self._vlines = []
        for p in self._plots():
            vline = pg.InfiniteLine(
                angle=90, movable=False,
                pen=pg.mkPen(color=(255, 200, 0, 150), width=1))
            vline.hide()
            p.addItem(vline, ignoreBounds=True)
            self._vlines.append(vline)
        self._crosshair_proxy = pg.SignalProxy(
            self.plot_widget.scene().sigMouseMoved,
            rateLimit=30, slot=self._on_crosshair_moved)

    def _on_crosshair_moved(self, evt):
        pos = evt[0]
        x = None
        for p in self._plots():
            vb = p.getViewBox()
            if vb.sceneBoundingRect().contains(pos):
                x = float(vb.mapSceneToView(pos).x())
                break
        if x is None:
            for v in self._vlines:
                v.hide()
            self.lbl_crosshair.setText("")
            return
        for v in self._vlines:
            v.setPos(x)
            v.show()
        self._update_readout(x)

    def _update_readout(self, x):
        parts = []
        for cid in sorted(self.buffers):
            b = self.buffers[cid]
            t = b["t"]
            n = len(t)
            if n == 0:
                continue
            i = int(np.searchsorted(t, x))
            if i >= n:
                i = n - 1
            elif i > 0 and abs(float(t[i - 1]) - x) < abs(float(t[i]) - x):
                i -= 1
            parts.append(
                "0x%X  pos=%.4f vel=%.4f tau=%.3f ff=%.3f"
                % (cid, float(b["pos"][i]), float(b["vel"][i]),
                   float(b["tau"][i]), float(b["ff_tau"][i])))
        self.lbl_crosshair.setText("  |  ".join(parts))

    # ------------------------------------------------------------------ 导出
    def export_png(self, path):
        return self.plot_widget.grab().save(path, "PNG")

    # ---------------------------------------------------------------- 供 Hmi
    @property
    def x_window(self):
        return self._x_window

    def n_curves(self):
        n = max(1, len(self.curves))
        return n * 2 if self.chk_show_cmd.isChecked() else n

    def plot_px(self):
        try:
            return max(400, int(self.plot_widget.width()))
        except Exception:
            return 800
