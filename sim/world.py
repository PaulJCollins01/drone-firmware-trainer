import random
from dataclasses import dataclass

ARENA_W = 20.0
ARENA_H = 15.0
ARENA_D = 10.0
SPAWN = (1.0, 1.0, 1.0)
GOAL_CENTER = (18.0, 13.0, 8.0)
GOAL_RADIUS = 0.75

@dataclass(frozen=True)
class Box:
    x: float
    y: float
    z: float
    w: float  # extent in X
    h: float  # extent in Y
    d: float  # extent in Z

    def contains_point(self, px: float, py: float, pz: float) -> bool:
        return (
            self.x <= px <= self.x + self.w
            and self.y <= py <= self.y + self.h
            and self.z <= pz <= self.z + self.d
        )

@dataclass
class World:
    arena_w: float
    arena_h: float
    arena_d: float
    obstacles: list[Box]
    spawn: tuple[float, float, float]
    goal_center: tuple[float, float, float]
    goal_radius: float

def _hand_crafted_seed_42() -> list[Box]:
    # Horizontal slab blocking direct XY path, requires climbing over or routing around.
    # Vertical pillar forces lateral dodge after the slab.
    return [
        Box(6.0,  6.0, 0.0, 4.0, 0.6, 4.0),   # wide slab blocking mid-path
        Box(11.0, 8.5, 2.0, 0.6, 4.5, 3.0),   # vertical pillar after the gap
        Box(3.0,  10.0, 1.0, 2.0, 0.6, 4.0),
        Box(14.0, 4.0,  0.0, 0.6, 3.0, 5.0),
        Box(8.5,  2.0,  3.0, 0.6, 3.0, 3.0),
    ]

def _random_obstacles(seed: int) -> list[Box]:
    rng = random.Random(seed)
    boxes: list[Box] = []
    attempts = 0
    while len(boxes) < 9 and attempts < 200:
        attempts += 1
        x = rng.uniform(3.0, 16.0)
        y = rng.uniform(2.0, 12.0)
        z = rng.uniform(0.0, 7.0)
        choice = rng.random()
        if choice < 0.33:
            w, h, d = rng.uniform(1.0, 3.0), 0.6, rng.uniform(1.0, 3.0)  # horizontal slab
        elif choice < 0.66:
            w, h, d = 0.6, rng.uniform(1.0, 3.0), rng.uniform(1.0, 3.0)  # Y-pillar
        else:
            w, h, d = rng.uniform(1.0, 3.0), rng.uniform(1.0, 3.0), 0.6  # Z-slab
        cand = Box(x, y, z, w, h, d)
        sx, sy, sz = SPAWN
        gx, gy, gz = GOAL_CENTER
        if cand.contains_point(sx, sy, sz) or cand.contains_point(gx, gy, gz):
            continue
        boxes.append(cand)
    return boxes

def build_world(seed: int) -> World:
    obstacles = _hand_crafted_seed_42() if seed == 42 else _random_obstacles(seed)
    return World(
        arena_w=ARENA_W,
        arena_h=ARENA_H,
        arena_d=ARENA_D,
        obstacles=obstacles,
        spawn=SPAWN,
        goal_center=GOAL_CENTER,
        goal_radius=GOAL_RADIUS,
    )
