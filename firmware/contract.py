import itertools
import math
from dataclasses import dataclass

# 26 directions: all 3x3x3 cube neighbors (6 face + 12 edge + 8 corner)
RAY_COUNT_3D = 26

_DIRS_RAW = [
    (dx, dy, dz)
    for dx, dy, dz in itertools.product((-1, 0, 1), repeat=3)
    if (dx, dy, dz) != (0, 0, 0)
]
assert len(_DIRS_RAW) == RAY_COUNT_3D

# Unit vectors corresponding to each ray index — same order used by the sensor module.
# Firmware imports this to interpret which direction each ray distance applies to.
RAY_DIRS: tuple[tuple[float, float, float], ...] = tuple(
    (dx / math.sqrt(dx*dx + dy*dy + dz*dz),
     dy / math.sqrt(dx*dx + dy*dy + dz*dz),
     dz / math.sqrt(dx*dx + dy*dy + dz*dz))
    for dx, dy, dz in _DIRS_RAW
)

@dataclass(frozen=True)
class SensorPacket:
    rays: tuple[float, ...]           # RAY_COUNT_3D distances, indexed by RAY_DIRS
    imu_accel: tuple[float, float, float]
    yaw: float                         # rotation around Z axis (radians)
    pitch: float                       # rotation around Y axis (radians)
    pos_estimate: tuple[float, float, float]
    dt: float

@dataclass(frozen=True)
class MotorCommand:
    thrust: tuple[float, float, float]

    @staticmethod
    def zero() -> "MotorCommand":
        return MotorCommand(thrust=(0.0, 0.0, 0.0))

    def clipped(self) -> "MotorCommand":
        fx, fy, fz = self.thrust
        return MotorCommand(thrust=(
            max(-1.0, min(1.0, fx)),
            max(-1.0, min(1.0, fy)),
            max(-1.0, min(1.0, fz)),
        ))
