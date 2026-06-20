import numpy as np
import pytest

# _project and _build_view are module-level helpers — import directly
from sim.renderer import _project, _build_view, _compute_chase_cam

W_PX, H_PX = 900, 650

CAM_DIST        = 6.0
CAM_Z_OFFSET    = 3.0
CAM_LOOK_AHEAD  = 1.0
CAM_ALPHA       = 0.05
CAM_SPEED_THRESH = 0.1

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
