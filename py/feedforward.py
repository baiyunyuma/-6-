"""前馈补偿模型：重力 / 库仑-粘性摩擦 / 惯性。

公式与 C++ feedforward_controller.c 保持一致，输出力矩单位 N·m，
直接加到 MIT 前馈力矩 (tau) 上。
"""
import math
import time

GRAVITY_ACCELERATION = 9.80665  # m/s^2


def gravity_torque(mass_kg, center_of_mass_m, angle_rad, angle_offset_rad, gravity_gain=1.0):
    """重力补偿力矩 (N·m)。angle_rad = 当前关节角度（弧度）。"""
    return (
        gravity_gain
        * mass_kg
        * GRAVITY_ACCELERATION
        * center_of_mass_m
        * math.cos(angle_rad + angle_offset_rad)
    )


def friction_torque(velocity_rad_s, coulomb_nm, viscous_coeff, tanh_slope):
    """库仑-粘性摩擦补偿力矩 (N·m)。velocity_rad_s = 当前角速度（rad/s）。"""
    return (
        coulomb_nm * math.tanh(tanh_slope * velocity_rad_s)
        + viscous_coeff * velocity_rad_s
    )


def inertia_torque(acceleration_rad_s2, inertia_kg_m2):
    """惯性补偿力矩 (N·m)。acceleration_rad_s2 = 当前角加速度（rad/s^2）。"""
    return inertia_kg_m2 * acceleration_rad_s2


class VelocityFilter:
    """速度一阶低通滤波器，用于惯性补偿的加速度计算。

    filtered = prev + alpha * (vel - prev)
    加速度    = (filtered - prev) / dt

    alpha ∈ [0, 1]：=1 不滤波（原样），越小滤波越强、越平滑。
    """

    def __init__(self, alpha: float = 1.0):
        self.alpha = alpha
        self._prev_filt = None
        self._prev_time = None

    def set_alpha(self, alpha: float):
        self.alpha = max(0.0, min(1.0, alpha))

    def update(self, vel: float) -> float:
        """输入当前速度，返回滤波后的加速度 (rad/s^2)。"""
        now = time.monotonic()
        alpha = max(0.0, min(1.0, self.alpha))
        if self._prev_filt is None:
            filt = vel
            accel = 0.0
        else:
            filt = self._prev_filt + alpha * (vel - self._prev_filt)
            dt = now - self._prev_time
            accel = 0.0 if dt <= 0 else (filt - self._prev_filt) / dt
        self._prev_filt = filt
        self._prev_time = now
        return accel
