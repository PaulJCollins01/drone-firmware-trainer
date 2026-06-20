import math
import time
from typing import Optional, Tuple
import numpy as np
import pygame

from firmware.contract import MotorCommand, SensorPacket, RAY_DIRS
from sim.physics import DroneState
from sim.world import World, Box

W_PX, H_PX = 900, 650
RENDER_HZ = 30.0
RENDER_DT = 1.0 / RENDER_HZ

BLACK  = (255, 255, 255)   # MC UI font white
WHITE  = (120, 167, 210)   # clear day sky
RED    = (220,  70,  20)   # redstone torch
GREEN  = (248, 204,  38)   # gold block beacon
BLUE   = ( 64, 164, 223)   # water/ice
GREY   = (123, 123, 123)   # stone block
DIM    = ( 74,  74,  74)   # bedrock/void edge

def _build_view(eye: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    f = target - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    M = np.eye(4)
    M[0, :3] = r;  M[0, 3] = -r @ eye
    M[1, :3] = u;  M[1, 3] = -u @ eye
    M[2, :3] = -f; M[2, 3] =  f @ eye
    return M

_F      = 1.0 / math.tan(math.radians(55.0) / 2)
_ASPECT = W_PX / H_PX

CAM_DIST         = 6.0
CAM_Z_OFFSET     = 3.0
CAM_LOOK_AHEAD   = 1.0
CAM_ALPHA        = 0.05
CAM_SPEED_THRESH = 0.1


def _compute_chase_cam(
    drone_pos: np.ndarray,
    vx: float,
    vy: float,
    cam_dir: np.ndarray,
) -> tuple:
    speed_xy = math.sqrt(vx * vx + vy * vy)
    if speed_xy > CAM_SPEED_THRESH:
        desired = np.array([vx / speed_xy, vy / speed_xy, 0.0])
        blended = (1.0 - CAM_ALPHA) * cam_dir + CAM_ALPHA * desired
        norm = np.linalg.norm(blended)
        new_dir = blended / norm if norm > 1e-6 else cam_dir
    else:
        new_dir = cam_dir
    eye    = drone_pos - new_dir * CAM_DIST + np.array([0.0, 0.0, CAM_Z_OFFSET])
    target = drone_pos + new_dir * CAM_LOOK_AHEAD
    return eye, target, new_dir

def _project(x: float, y: float, z: float, view: np.ndarray) -> Optional[Tuple[int, int]]:
    p = view @ np.array([x, y, z, 1.0])
    if p[2] >= -0.1:
        return None  # behind camera
    sx = int(( _F / _ASPECT * p[0] / (-p[2]) + 1) * W_PX / 2)
    sy = int((1 - _F              * p[1] / (-p[2])) * H_PX / 2)
    return sx, sy

def _draw_line(surf, color, a3, b3, view: np.ndarray, width=1) -> None:
    pa = _project(*a3, view)
    pb = _project(*b3, view)
    if pa and pb:
        pygame.draw.line(surf, color, pa, pb, width)

def _box_edges(b: Box):
    x, y, z, w, h, d = b.x, b.y, b.z, b.w, b.h, b.d
    c = [
        (x,     y,     z    ), (x + w, y,     z    ),
        (x + w, y + h, z    ), (x,     y + h, z    ),
        (x,     y,     z + d), (x + w, y,     z + d),
        (x + w, y + h, z + d), (x,     y + h, z + d),
    ]
    return [
        (c[0], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[0]),  # bottom face
        (c[4], c[5]), (c[5], c[6]), (c[6], c[7]), (c[7], c[4]),  # top face
        (c[0], c[4]), (c[1], c[5]), (c[2], c[6]), (c[3], c[7]),  # verticals
    ]

class Renderer:
    def __init__(self, world: World):
        pygame.init()
        self.world = world
        self.screen = pygame.display.set_mode((W_PX, H_PX))
        pygame.display.set_caption("Drone Firmware Trainer 3D")
        self.font     = pygame.font.SysFont("consolas", 14)
        self.font_big = pygame.font.SysFont("consolas", 28, bold=True)
        self._last_render = 0.0
        self._cam_dir = np.array([1.0, 0.0, 0.0])

    def draw(self, drone: DroneState, packet: SensorPacket, cmd: MotorCommand) -> None:
        now = time.perf_counter()
        if now - self._last_render < RENDER_DT:
            return
        self._last_render = now

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                raise SystemExit(0)

        self.screen.fill(WHITE)

        drone_pos = np.array([drone.x, drone.y, drone.z])
        eye, target, self._cam_dir = _compute_chase_cam(
            drone_pos, drone.vx, drone.vy, self._cam_dir
        )
        view = _build_view(eye, target, np.array([0.0, 0.0, 1.0]))

        # arena bounding box
        W, H, D = self.world.arena_w, self.world.arena_h, self.world.arena_d
        arena = Box(0.0, 0.0, 0.0, W, H, D)
        for a, b in _box_edges(arena):
            _draw_line(self.screen, DIM, a, b, view)

        # obstacles
        for obs in self.world.obstacles:
            for a, b in _box_edges(obs):
                _draw_line(self.screen, GREY, a, b, view, 2)

        # goal marker: three crossing circles approximated as dot + crosshair lines
        gx, gy, gz = self.world.goal_center
        r = self.world.goal_radius
        for a, b in [
            ((gx-r, gy, gz), (gx+r, gy, gz)),
            ((gx, gy-r, gz), (gx, gy+r, gz)),
            ((gx, gy, gz-r), (gx, gy, gz+r)),
        ]:
            _draw_line(self.screen, GREEN, a, b, view, 3)
        gp = _project(gx, gy, gz, view)
        if gp:
            pygame.draw.circle(self.screen, GREEN, gp, 8, 2)

        # rays from drone
        dx, dy, dz = drone.x, drone.y, drone.z
        for i, ray in enumerate(packet.rays):
            rdx, rdy, rdz = RAY_DIRS[i]
            end = (dx + rdx * ray, dy + rdy * ray, dz + rdz * ray)
            _draw_line(self.screen, BLUE, (dx, dy, dz), end, view)

        # drone dot
        dp = _project(dx, dy, dz, view)
        if dp:
            pygame.draw.circle(self.screen, RED, dp, 7)

        # HUD
        hud = self.font.render(
            f"t={drone.t:5.2f}s  "
            f"pos=({drone.x:4.1f},{drone.y:4.1f},{drone.z:4.1f})  "
            f"thr=({cmd.thrust[0]:+.2f},{cmd.thrust[1]:+.2f},{cmd.thrust[2]:+.2f})",
            True, BLACK,
        )
        self.screen.blit(hud, (10, 10))
        pygame.display.flip()

    def hold(self, banner: str) -> None:
        text = self.font_big.render(banner, True, BLACK)
        hint = self.font.render("press any key or close window to exit", True, BLACK)
        tw, th = text.get_size()
        hw, hh = hint.get_size()
        pad = 14
        box_w = max(tw, hw) + pad * 2
        box_h = th + hh + pad * 3
        box_x = (W_PX - box_w) // 2
        box_y = (H_PX - box_h) // 2
        try:
            pygame.draw.rect(self.screen, WHITE, (box_x, box_y, box_w, box_h))
            pygame.draw.rect(self.screen, BLACK, (box_x, box_y, box_w, box_h), 2)
            self.screen.blit(text, (box_x + (box_w - tw) // 2, box_y + pad))
            self.screen.blit(hint, (box_x + (box_w - hw) // 2, box_y + pad * 2 + th))
            pygame.display.flip()
            while True:
                for ev in pygame.event.get():
                    if ev.type in (pygame.QUIT, pygame.KEYDOWN, pygame.MOUSEBUTTONDOWN):
                        return
                time.sleep(0.02)
        except pygame.error:
            return

    def close(self) -> None:
        pygame.quit()
