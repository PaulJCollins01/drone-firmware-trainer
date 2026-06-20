import numpy as np

# _project and _build_view are module-level helpers — import directly
from sim.renderer import _project, _build_view, _compute_chase_cam, CAM_DIST, CAM_Z_OFFSET, CAM_LOOK_AHEAD, CAM_ALPHA, CAM_SPEED_THRESH
from sim.renderer import WHITE, DIM, GREY, GREEN, BLUE, RED, BLACK
from sim.renderer import GRASS, _draw_ground

def _make_view():
    eye    = np.array([-10.0, -14.0, 18.0])
    target = np.array([10.0,   7.5,   5.0])
    up     = np.array([0.0,    0.0,   1.0])
    return _build_view(eye, target, up)

def test_project_accepts_explicit_view():
    view = _make_view()
    result = _project(10.0, 7.5, 5.0, view)
    # center of arena should project near center of screen
    assert result is not None
    sx, sy = result
    assert 300 < sx < 600
    assert 200 < sy < 450

def test_project_behind_camera_returns_none():
    view = _make_view()
    # a point behind the camera eye should return None
    result = _project(-10.0, -14.0, 19.0, view)
    assert result is None

def test_chase_cam_heading_updates_when_moving():
    drone_pos = np.array([5.0, 5.0, 5.0])
    cam_dir   = np.array([1.0, 0.0, 0.0])
    # drone moving in +Y direction
    eye, target, new_dir = _compute_chase_cam(drone_pos, 0.0, 1.0, cam_dir)
    # new_dir should have lerped toward (0,1,0)
    assert new_dir[1] > 0.0
    assert new_dir[0] < 1.0  # x component reduced

def test_chase_cam_heading_frozen_when_stationary():
    drone_pos = np.array([5.0, 5.0, 5.0])
    cam_dir   = np.array([1.0, 0.0, 0.0])
    eye, target, new_dir = _compute_chase_cam(drone_pos, 0.0, 0.0, cam_dir)
    np.testing.assert_array_almost_equal(new_dir, cam_dir)

def test_chase_cam_eye_is_behind_and_above():
    drone_pos = np.array([5.0, 5.0, 5.0])
    cam_dir   = np.array([1.0, 0.0, 0.0])  # facing +X
    eye, target, _ = _compute_chase_cam(drone_pos, 1.0, 0.0, cam_dir)
    # eye should be behind (lower X) and above (higher Z)
    assert eye[0] < drone_pos[0]   # behind in X
    assert eye[2] > drone_pos[2]   # above in Z
    assert abs(eye[2] - (drone_pos[2] + CAM_Z_OFFSET)) < 0.01

def test_chase_cam_target_is_ahead_of_drone():
    drone_pos = np.array([5.0, 5.0, 5.0])
    cam_dir   = np.array([1.0, 0.0, 0.0])
    eye, target, _ = _compute_chase_cam(drone_pos, 1.0, 0.0, cam_dir)
    # target should be ahead of drone in +X
    assert target[0] > drone_pos[0]
    assert abs(target[0] - (drone_pos[0] + CAM_LOOK_AHEAD)) < 0.01

def test_minecraft_overworld_palette():
    assert WHITE == (120, 167, 210), f"Expected sky blue, got {WHITE}"
    assert DIM   == (74,  74,  74),  f"Expected bedrock dark, got {DIM}"
    assert GREY  == (123, 123, 123), f"Expected stone grey, got {GREY}"
    assert GREEN == (248, 204, 38),  f"Expected gold beacon, got {GREEN}"
    assert BLUE  == (64,  164, 223), f"Expected water blue, got {BLUE}"
    assert RED   == (220, 70,  20),  f"Expected redstone orange, got {RED}"
    assert BLACK == (255, 255, 255), f"Expected MC UI white, got {BLACK}"

def test_grass_color():
    assert GRASS == (89, 125, 39), f"Expected Minecraft grass green, got {GRASS}"

def test_draw_ground_skips_when_corner_behind_camera():
    # A view matrix where all ground corners are behind the camera (z >= -0.1)
    # Use identity — points at z=0 project to p[2]=0 which is >= -0.1, so all return None
    view = np.eye(4)
    # Should not raise even when all corners clip
    _draw_ground(None, view, 20.0, 15.0)
