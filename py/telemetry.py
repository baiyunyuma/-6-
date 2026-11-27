"""定长 numpy 环形缓冲 + 峰值保持降采样。

从 DM_DJI_TOOL_V1_src 的 motor_worker.py 移植（只保留达妙通道）。

纯 numpy，不 import Qt、不 import SDK —— 可以直接在普通 Python 里单测。
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

#: 每帧反馈记录的量。顺序即 push 的顺序，不能随便改。
TELEM_CHANNELS = ("pos", "vel", "tau", "ff_tau", "cmd_q", "cmd_dq", "cmd_tau")

#: 缓冲容量。1 kHz 下约 24 s，250 Hz 下约 98 s。
#: `cap // 2`（写满后的左移量）= 12288 = 2^12 × 3，能被 ≤4096 的任意 2 的幂整除 ——
#: 峰值分桶的桶边界锚定在绝对采样序号上，左移量必须是桶宽的整数倍，
#: 否则每写满半个缓冲，整条曲线就会重新挑一次点（看起来在闪）。
TELEM_CAPACITY = 24576


def _peak_indices_block(block, buckets: int):
    """把 (n, C) 的块分成 buckets 段，每段取两个下标（min/max）。

    取哪一段通道的极值？取该段内 `max-min`（波动）最大的通道：
    位置平缓、力矩出尖刺的时刻，尖刺也一定被保留下来。

    返回的下标严格递增，长度 ≤ 2*buckets+1，且一定包含最后一个采样点。
    不需要降采样时返回 None。
    """
    n, c = block.shape
    buckets = int(buckets)
    if buckets < 1 or n <= 2 * buckets:
        return None
    k = n // buckets
    m = n // k
    if k < 2 or m < 2:
        return None
    # 先把 (n, C) 转成 (C, n) 的连续内存再归约：直接在 (m, k, C) 视图上
    # 按 axis=1 归约，numpy 走不了向量化路径，实测慢 30 倍以上 ——
    # 而这把 CPU 是花在 _sample_lock 里的，CAN 接收线程要跟着等。
    seg = np.ascontiguousarray(block[:m * k].T).reshape(c, m, k)
    lo = seg.min(axis=2)
    hi = seg.max(axis=2)
    best = (hi - lo).argmax(axis=0)
    i_lo = seg.argmin(axis=2)
    i_hi = seg.argmax(axis=2)
    cols = np.arange(m, dtype=np.int64)
    a = i_lo[best, cols]
    b = i_hi[best, cols]
    idx = np.empty(m * 2, dtype=np.int64)
    idx[0::2] = np.arange(m, dtype=np.int64) * k + np.minimum(a, b)
    idx[1::2] = np.arange(m, dtype=np.int64) * k + np.maximum(a, b)
    if idx[-1] != n - 1:
        idx = np.append(idx, n - 1)
    # 平段（min == max）会取到同一个下标，去重后时间轴才严格递增
    keep = np.empty(idx.size, dtype=bool)
    keep[0] = True
    np.not_equal(idx[1:], idx[:-1], out=keep[1:])
    return idx[keep]


def _pow2_near(v: int) -> int:
    """最接近 v 的 2 的幂（至少 2）。

    把桶宽钉在稳定档位上：桶宽取任意整数时，跨度抖动一点就会换档，
    一换档整条曲线重新挑点。钉到 2 的幂后，跨度 ±40% 的波动都不换档。
    """
    v = int(v)
    if v <= 2:
        return 2
    lo = 1 << (v.bit_length() - 1)
    hi = lo << 1
    return lo if (v - lo) <= (hi - v) else hi


class TelemetryBuffer:
    """定长 numpy 环形缓冲（写满时左移一半，保证 view() 总是连续内存）。"""

    __slots__ = ("channels", "cap", "n", "base", "idx", "t", "data")

    def __init__(self, channels: Tuple[str, ...] = TELEM_CHANNELS,
                 cap: int = TELEM_CAPACITY):
        self.channels = tuple(channels)
        self.cap = int(cap)
        self.n = 0
        #: self.data[0] 对应的绝对采样序号（自会话开始累计写入的点数）。
        self.base = 0
        self.idx = {c: k for k, c in enumerate(self.channels)}
        self.t = np.zeros(self.cap, dtype=np.float64)
        self.data = np.zeros((self.cap, len(self.channels)), dtype=np.float32)

    def clear(self) -> None:
        self.n = 0
        self.base = 0

    def _make_room(self) -> None:
        if self.n < self.cap:
            return
        keep = self.cap // 2
        align = 1 << 12
        if keep >= align:
            keep -= keep % align
        else:
            keep = max(1, keep)
        self.t[:keep] = self.t[keep:]
        self.data[:keep] = self.data[keep:]
        self.n = keep
        self.base += keep

    def push(self, t: float, values) -> None:
        """写一个采样点。values 必须按 self.channels 的顺序给出。"""
        self._make_room()
        i = self.n
        self.t[i] = t
        self.data[i] = values
        self.n = i + 1

    def view(self, max_points=None, tail=None):
        """返回 (t, {通道: 一维连续数组})，永远是拷贝。

        * ``tail``：只取最近这么多点（曲线只看得到当前视窗）；
        * ``max_points``：按峰值保持降到不超过这个点数。

        分桶锚定在绝对采样序号（self.base）上，不对滑动窗口分桶 ——
        否则窗口每前移一格，桶边界就跟着移，曲线上的极值位置会不停跳动。
        """
        n = self.n
        start = 0
        if tail is not None and tail > 0 and n > tail:
            start = n - int(tail)
        sl = slice(start, n)

        if max_points is not None and max_points > 0 and (n - start) > max_points:
            buckets = max(1, int(max_points) // 2)
            span_ref = int(tail) if (tail is not None and tail > 0) else (n - start)
            stride = _pow2_near(max(2, -(-span_ref // buckets)))
            abs_start = self.base + start
            s0 = max(0, (abs_start // stride) * stride - self.base)
            q = (n - s0) // stride
            sel = None
            if q >= 2:
                sel = _peak_indices_block(self.data[s0:s0 + q * stride], q)
                if sel is not None and sel.size:
                    last = (n - 1) - s0
                    if sel[-1] != last:
                        sel = np.append(sel, last)
            if sel is not None:
                blk = slice(s0, n)
                t = self.t[blk][sel]
                data = {c: self.data[blk, k][sel] for c, k in self.idx.items()}
                return t, data

        t = self.t[sl].copy()
        data = {c: np.ascontiguousarray(self.data[sl, k])
                for c, k in self.idx.items()}
        return t, data
