from dataclasses import dataclass
from sim.world import Box

MASS = 0.5
DRAG = 0.4
GRAVITY = 0.4  # m/s² downward on Z; must be < 1.0 so max thrust (1.0) can overcome it

@dataclass
class DroneState:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    t: float

def make_drone(spawn: tuple[float, float, float]) -> DroneState:
    return DroneState(x=spawn[0], y=spawn[1], z=spawn[2], vx=0.0, vy=0.0, vz=0.0, t=0.0)

def step_physics(state: DroneState, thrust: tuple[float, float, float], dt: float) -> None:
    fx = max(-1.0, min(1.0, thrust[0]))
    fy = max(-1.0, min(1.0, thrust[1]))
    fz = max(-1.0, min(1.0, thrust[2]))
    ax = (fx - DRAG * state.vx) / MASS
    ay = (fy - DRAG * state.vy) / MASS
    az = (fz - DRAG * state.vz - GRAVITY) / MASS  # gravity pulls down
    state.vx += ax * dt
    state.vy += ay * dt
    state.vz += az * dt
    state.x += state.vx * dt
    state.y += state.vy * dt
    state.z += state.vz * dt
    state.t += dt

def is_inside_any(boxes: list[Box], px: float, py: float, pz: float) -> bool:
    return any(b.contains_point(px, py, pz) for b in boxes)

def min_clearance(boxes: list[Box], px: float, py: float, pz: float) -> float:
    if not boxes:
        return float("inf")
    best = float("inf")
    for b in boxes:
        if b.contains_point(px, py, pz):
            face_dx = min(px - b.x, b.x + b.w - px)
            face_dy = min(py - b.y, b.y + b.h - py)
            face_dz = min(pz - b.z, b.z + b.d - pz)
            d = -min(face_dx, face_dy, face_dz)
        else:
            cx = max(b.x, min(px, b.x + b.w))
            cy = max(b.y, min(py, b.y + b.h))
            cz = max(b.z, min(pz, b.z + b.d))
            d = ((px - cx) ** 2 + (py - cy) ** 2 + (pz - cz) ** 2) ** 0.5
        if d < best:
            best = d
    return best
