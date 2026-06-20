import numpy as np
import pytest

# _project and _build_view are module-level helpers — import directly
from sim.renderer import _project, _build_view

W_PX, H_PX = 900, 650

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
