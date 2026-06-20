import math
from firmware.contract import SensorPacket, MotorCommand, RAY_DIRS

# Find the ray index whose direction is closest to +X (1,0,0) — the "forward" axis.
# RAY_DIRS[21] == (1.0, 0.0, 0.0) exactly, but we derive it programmatically so
# this stays correct even if the ordering in contract.py ever changes.
_FORWARD_RAY_IDX = max(range(len(RAY_DIRS)), key=lambda i: RAY_DIRS[i][0])

GOAL_X, GOAL_Y, GOAL_Z = 18.0, 13.0, 8.0
HOVER_GAIN = 0.5  # minimum upward thrust to fight gravity (must exceed GRAVITY=0.4)

class Firmware:
    def __init__(self) -> None:
        pass

    def step(self, sensors: SensorPacket) -> MotorCommand:
        px, py, pz = sensors.pos_estimate
        dx, dy, dz = GOAL_X - px, GOAL_Y - py, GOAL_Z - pz
        norm = math.sqrt(dx*dx + dy*dy + dz*dz) or 1.0
        ux, uy, uz = dx / norm, dy / norm, dz / norm

        # naive: only checks the +X ray, ignores 25 others
        forward = sensors.rays[_FORWARD_RAY_IDX]

        if forward < 1.0:
            fx, fy, fz = -ux * 0.3, -uy * 0.3, -uz * 0.3
        else:
            fx, fy, fz = ux * 0.8, uy * 0.8, uz * 0.8

        fz += HOVER_GAIN  # still needs to fight gravity or it falls immediately

        return MotorCommand(thrust=(fx, fy, fz)).clipped()
