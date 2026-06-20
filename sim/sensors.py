import itertools
import math
import numpy as np
from firmware.contract import SensorPacket, RAY_COUNT_3D
from sim.physics import DroneState
from sim.world import Box

RAY_MAX = 4.0
RAY_STEP = 0.05

RAY_NOISE  = 0.03
ACCEL_NOISE = 0.05
POS_NOISE  = 0.05
YAW_NOISE  = 0.02
PITCH_NOISE = 0.02
QUANT = 0.01

# 26 unit vectors pointing toward every neighbor in a 3x3x3 grid (excluding center).
# Fixed world-space directions — firmware always knows which index points which way.
_DIRS_RAW = [
    (dx, dy, dz)
    for dx, dy, dz in itertools.product((-1, 0, 1), repeat=3)
    if (dx, dy, dz) != (0, 0, 0)
]
assert len(_DIRS_RAW) == RAY_COUNT_3D
RAY_DIRS: tuple[tuple[float, float, float], ...] = tuple(
    (dx / math.sqrt(dx*dx + dy*dy + dz*dz),
     dy / math.sqrt(dx*dx + dy*dy + dz*dz),
     dz / math.sqrt(dx*dx + dy*dy + dz*dz))
    for dx, dy, dz in _DIRS_RAW
)

def _q(v: float) -> float:
    return round(v / QUANT) * QUANT

def _ray_cast(px: float, py: float, pz: float,
              dx: float, dy: float, dz: float,
              boxes: list[Box]) -> float:
    d = 0.0
    while d < RAY_MAX:
        d += RAY_STEP
        for b in boxes:
            if b.contains_point(px + dx * d, py + dy * d, pz + dz * d):
                return d
    return RAY_MAX

def build_sensor_packet(
    state: DroneState,
    boxes: list[Box],
    dt: float,
    rng: np.random.Generator,
) -> SensorPacket:
    speed_xy = math.hypot(state.vx, state.vy)
    yaw_true   = math.atan2(state.vy, state.vx) if speed_xy or state.vz else 0.0
    pitch_true = math.atan2(-state.vz, speed_xy) if speed_xy or state.vz else 0.0

    rays_noisy = tuple(
        _q(max(0.0, min(RAY_MAX,
            _ray_cast(state.x, state.y, state.z, dx, dy, dz, boxes)
            + float(rng.normal(0.0, RAY_NOISE)))))
        for dx, dy, dz in RAY_DIRS
    )
    accel = (
        _q(float(rng.normal(0.0, ACCEL_NOISE))),
        _q(float(rng.normal(0.0, ACCEL_NOISE))),
        _q(float(rng.normal(0.0, ACCEL_NOISE))),
    )
    yaw   = _q(yaw_true   + float(rng.normal(0.0, YAW_NOISE)))
    pitch = _q(pitch_true + float(rng.normal(0.0, PITCH_NOISE)))
    pos = (
        _q(state.x + float(rng.normal(0.0, POS_NOISE))),
        _q(state.y + float(rng.normal(0.0, POS_NOISE))),
        _q(state.z + float(rng.normal(0.0, POS_NOISE))),
    )
    return SensorPacket(
        rays=rays_noisy,
        imu_accel=accel,
        yaw=yaw,
        pitch=pitch,
        pos_estimate=pos,
        dt=dt,
    )
