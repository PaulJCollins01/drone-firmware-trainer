import math
from typing import Optional
from firmware.contract import SensorPacket, MotorCommand, RAY_DIRS

GOAL_X, GOAL_Y, GOAL_Z = 18.0, 13.0, 8.0
RAY_MAX = 4.0
REPULSION_GAIN  = 1.5
ATTRACTION_GAIN = 0.8
HOVER_GAIN      = 0.5   # constant upward bias to counteract gravity (must exceed GRAVITY=0.4)

SMOOTH_ALPHA        = 0.02
STUCK_CHECK_INTERVAL = 50
STUCK_GUARD_TICKS   = 250
STUCK_DISP_THR      = 0.25
ESCAPE_TICKS        = 200
ESCAPE_PERP_GAIN    = 0.5  # reduced: large values cause out-of-bounds excursions

# Arena boundary repulsion — keeps drone away from walls the rays can't see
ARENA_W, ARENA_H, ARENA_D = 20.0, 15.0, 10.0
WALL_MARGIN = 2.0   # meters from wall where repulsion starts
WALL_GAIN   = 1.2

def _wall_push(dist: float) -> float:
    if dist >= WALL_MARGIN:
        return 0.0
    return WALL_GAIN * (1.0 - dist / WALL_MARGIN) ** 2

def _wall_repulsion(px: float, py: float, pz: float):
    rx = _wall_push(px) - _wall_push(ARENA_W - px)
    ry = _wall_push(py) - _wall_push(ARENA_H - py)
    rz = _wall_push(pz) - _wall_push(ARENA_D - pz)
    return rx, ry, rz

class Firmware:
    def __init__(self) -> None:
        self._last_thrust = (0.0, 0.0, 0.0)
        self._smooth_px: Optional[float] = None
        self._smooth_py: Optional[float] = None
        self._smooth_pz: Optional[float] = None
        self._checkpoint_px = 0.0
        self._checkpoint_py = 0.0
        self._checkpoint_pz = 0.0
        self._tick = 0
        self._escape_remaining = 0
        self._escape_sign = -1.0

    def step(self, sensors: SensorPacket) -> MotorCommand:
        px, py, pz = sensors.pos_estimate
        self._tick += 1

        # IIR position smoother
        if self._smooth_px is None:
            self._smooth_px, self._smooth_py, self._smooth_pz = px, py, pz
            self._checkpoint_px, self._checkpoint_py, self._checkpoint_pz = px, py, pz
        else:
            self._smooth_px += SMOOTH_ALPHA * (px - self._smooth_px)
            self._smooth_py += SMOOTH_ALPHA * (py - self._smooth_py)
            self._smooth_pz += SMOOTH_ALPHA * (pz - self._smooth_pz)

        # stuck detection in 3D
        if self._tick % STUCK_CHECK_INTERVAL == 0 and self._tick >= STUCK_GUARD_TICKS:
            disp = math.sqrt(
                (self._smooth_px - self._checkpoint_px) ** 2 +
                (self._smooth_py - self._checkpoint_py) ** 2 +
                (self._smooth_pz - self._checkpoint_pz) ** 2
            )
            if disp < STUCK_DISP_THR and self._escape_remaining == 0:
                self._escape_remaining = ESCAPE_TICKS
                self._escape_sign *= -1.0
            self._checkpoint_px = self._smooth_px
            self._checkpoint_py = self._smooth_py
            self._checkpoint_pz = self._smooth_pz

        # attraction toward goal in 3D
        dx, dy, dz = GOAL_X - px, GOAL_Y - py, GOAL_Z - pz
        norm = math.sqrt(dx*dx + dy*dy + dz*dz) or 1.0
        ux, uy, uz = dx / norm, dy / norm, dz / norm
        ax = ATTRACTION_GAIN * ux
        ay = ATTRACTION_GAIN * uy
        az = ATTRACTION_GAIN * uz + HOVER_GAIN  # hover bias fights gravity

        # repulsion from all 26 rays
        rx = ry = rz = 0.0
        for i, ray in enumerate(sensors.rays):
            if ray >= RAY_MAX * 0.95:
                continue
            rdx, rdy, rdz = RAY_DIRS[i]
            strength = REPULSION_GAIN * (1.0 - ray / RAY_MAX) ** 2
            rx -= rdx * strength
            ry -= rdy * strength
            rz -= rdz * strength

        wrx, wry, wrz = _wall_repulsion(px, py, pz)
        fx_raw = ax + rx + wrx
        fy_raw = ay + ry + wry
        fz_raw = az + rz + wrz

        # escape: perpendicular-to-goal bias in 3D
        if self._escape_remaining > 0:
            # cross goal direction with world-up (0,0,1); fall back to (1,0,0) if parallel
            if abs(uz) < 0.9:
                perp_x, perp_y, perp_z = -uy, ux, 0.0
            else:
                perp_x, perp_y, perp_z = 0.0, -uz, uy
            pnorm = math.sqrt(perp_x*perp_x + perp_y*perp_y + perp_z*perp_z) or 1.0
            perp_x /= pnorm
            perp_y /= pnorm
            perp_z /= pnorm
            fx_raw += ESCAPE_PERP_GAIN * perp_x * self._escape_sign
            fy_raw += ESCAPE_PERP_GAIN * perp_y * self._escape_sign
            fz_raw += ESCAPE_PERP_GAIN * perp_z * self._escape_sign
            self._escape_remaining -= 1

        # low-pass smoothing
        fx = 0.7 * fx_raw + 0.3 * self._last_thrust[0]
        fy = 0.7 * fy_raw + 0.3 * self._last_thrust[1]
        fz = 0.7 * fz_raw + 0.3 * self._last_thrust[2]
        self._last_thrust = (fx, fy, fz)

        return MotorCommand(thrust=(fx, fy, fz)).clipped()
