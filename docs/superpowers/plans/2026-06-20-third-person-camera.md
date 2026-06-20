# Third-Person Chase Camera Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the static fixed camera in `sim/renderer.py` with a smoothed third-person chase cam that sits above and behind the drone relative to its movement direction.

**Architecture:** A `_compute_chase_cam` helper function encapsulates all camera math (heading lerp, eye/target computation) so it can be unit tested independently of pygame. `_project` and `_draw_line` are updated to accept an explicit `view` matrix argument instead of reading the module-level constant. `Renderer.draw()` rebuilds the view matrix each frame from the drone's current position and velocity.

**Tech Stack:** Python 3.9, numpy, pygame 2.5.2

## Global Constraints

- Only `sim/renderer.py` and its tests are modified — no other files change.
- Python 3.9 compatible (no `match`, no `X | Y` union syntax).
- All existing sweep tests must continue to pass (camera is render-only).

---

### Task 1: Make `_project` and `_draw_line` accept an explicit view matrix

**Files:**
- Modify: `sim/renderer.py`
- Test: `tests/test_renderer_camera.py` (create)

**Interfaces:**
- Produces:
  - `_project(x, y, z, view: np.ndarray) -> Optional[Tuple[int, int]]`
  - `_draw_line(surf, color, a3, b3, view: np.ndarray, width=1) -> None`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_renderer_camera.py
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
```

- [ ] **Step 2: Run test to verify it fails**

```
cd /Users/paulcollins/projects/stockham-hackathon
.venv/bin/python -m pytest tests/test_renderer_camera.py -v
```

Expected: `TypeError` — `_project()` takes 3 positional arguments but 4 were given (or ImportError if `_project` not exported).

- [ ] **Step 3: Update `_project` to accept `view` parameter**

In `sim/renderer.py`, replace:

```python
_VIEW = _build_view(_CAM_POS, _CAM_TARGET, _CAM_UP)
_F    = 1.0 / math.tan(math.radians(_FOV_DEG) / 2)
_ASPECT = W_PX / H_PX

def _project(x: float, y: float, z: float) -> Optional[Tuple[int, int]]:
    p = _VIEW @ np.array([x, y, z, 1.0])
    if p[2] >= -0.1:
        return None  # behind camera
    sx = int(( _F / _ASPECT * p[0] / (-p[2]) + 1) * W_PX / 2)
    sy = int((1 - _F              * p[1] / (-p[2])) * H_PX / 2)
    return sx, sy

def _draw_line(surf, color, a3, b3, width=1) -> None:
    pa = _project(*a3)
    pb = _project(*b3)
    if pa and pb:
        pygame.draw.line(surf, color, pa, pb, width)
```

with:

```python
_F      = 1.0 / math.tan(math.radians(55.0) / 2)
_ASPECT = W_PX / H_PX

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
```

Also remove the four old camera constants at the top of the file:
```python
# DELETE these four lines:
_CAM_POS    = np.array([-10.0, -14.0, 18.0])
_CAM_TARGET = np.array([10.0,   7.5,   5.0])
_CAM_UP     = np.array([0.0,    0.0,   1.0])
_FOV_DEG    = 55.0
```

- [ ] **Step 4: Fix all `_draw_line` and `_project` call sites in `Renderer.draw()`**

Every call to `_draw_line` in `draw()` needs a `view` argument. Temporarily pass a hardcoded view so the file stays runnable while Task 2 is incomplete. Add this just before the arena drawing in `draw()`:

```python
_tmp_view = _build_view(
    np.array([-10.0, -14.0, 18.0]),
    np.array([10.0,   7.5,   5.0]),
    np.array([0.0,    0.0,   1.0]),
)
```

Then add `, _tmp_view` to every `_draw_line(...)` call and `, _tmp_view` to the two direct `_project(...)` calls (goal marker and drone dot). There are 5 call sites total:

```python
# arena bounding box
for a, b in _box_edges(arena):
    _draw_line(self.screen, DIM, a, b, _tmp_view)

# obstacles
for a, b in _box_edges(obs):
    _draw_line(self.screen, GREY, a, b, _tmp_view, 2)

# goal crosshair lines
_draw_line(self.screen, GREEN, a, b, _tmp_view, 3)

# goal circle
gp = _project(gx, gy, gz, _tmp_view)

# rays
_draw_line(self.screen, BLUE, (dx, dy, dz), end, _tmp_view)

# drone dot
dp = _project(dx, dy, dz, _tmp_view)
```

- [ ] **Step 5: Run tests to verify they pass**

```
.venv/bin/python -m pytest tests/test_renderer_camera.py -v
```

Expected: both tests PASS.

- [ ] **Step 6: Verify sweep still works**

```
.venv/bin/python -m pytest tests/ -v
```

Expected: all tests PASS.

- [ ] **Step 7: Commit**

```bash
git add sim/renderer.py tests/test_renderer_camera.py
git commit -m "refactor(renderer): _project/_draw_line accept explicit view matrix"
```

---

### Task 2: Add chase camera logic

**Files:**
- Modify: `sim/renderer.py`
- Test: `tests/test_renderer_camera.py`

**Interfaces:**
- Consumes:
  - `_build_view(eye, target, up) -> np.ndarray` (unchanged)
  - `_project(x, y, z, view: np.ndarray)` (from Task 1)
  - `_draw_line(surf, color, a3, b3, view: np.ndarray, width)` (from Task 1)
- Produces:
  - `_compute_chase_cam(drone_pos: np.ndarray, vx: float, vy: float, cam_dir: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]`
    - returns `(eye, target, new_cam_dir)`

- [ ] **Step 1: Write the failing tests**

Add to `tests/test_renderer_camera.py`:

```python
from sim.renderer import _compute_chase_cam

CAM_DIST        = 6.0
CAM_Z_OFFSET    = 3.0
CAM_LOOK_AHEAD  = 1.0
CAM_ALPHA       = 0.05
CAM_SPEED_THRESH = 0.1

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
```

- [ ] **Step 2: Run tests to verify they fail**

```
.venv/bin/python -m pytest tests/test_renderer_camera.py::test_chase_cam_heading_updates_when_moving -v
```

Expected: `ImportError: cannot import name '_compute_chase_cam'`

- [ ] **Step 3: Add chase camera constants and `_compute_chase_cam` to `sim/renderer.py`**

Add these constants after `_ASPECT`:

```python
CAM_DIST         = 6.0
CAM_Z_OFFSET     = 3.0
CAM_LOOK_AHEAD   = 1.0
CAM_ALPHA        = 0.05
CAM_SPEED_THRESH = 0.1
```

Add this function after the constants:

```python
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
```

- [ ] **Step 4: Run the new tests to verify they pass**

```
.venv/bin/python -m pytest tests/test_renderer_camera.py -v
```

Expected: all 6 tests PASS.

- [ ] **Step 5: Wire chase camera into `Renderer.__init__` and `draw()`**

In `Renderer.__init__`, add after `self._last_render = 0.0`:

```python
self._cam_dir = np.array([1.0, 0.0, 0.0])
```

In `Renderer.draw()`, replace the temporary `_tmp_view` block added in Task 1:

```python
_tmp_view = _build_view(
    np.array([-10.0, -14.0, 18.0]),
    np.array([10.0,   7.5,   5.0]),
    np.array([0.0,    0.0,   1.0]),
)
```

with:

```python
drone_pos = np.array([drone.x, drone.y, drone.z])
eye, target, self._cam_dir = _compute_chase_cam(
    drone_pos, drone.vx, drone.vy, self._cam_dir
)
view = _build_view(eye, target, np.array([0.0, 0.0, 1.0]))
```

Then rename every `_tmp_view` reference in that method to `view`. There are 5 of them (arena, obstacles, goal lines, goal circle, rays, drone dot).

- [ ] **Step 6: Run all tests**

```
.venv/bin/python -m pytest tests/ -v
```

Expected: all tests PASS.

- [ ] **Step 7: Smoke-test the renderer visually**

Run in Terminal (requires display):

```
cd ~/projects/stockham-hackathon
.venv/bin/python -m harness.run --firmware v2 --seed 42 --render
```

Verify: camera starts behind the drone, rotates smoothly as the drone changes direction, and doesn't spin when hovering.

- [ ] **Step 8: Commit**

```bash
git add sim/renderer.py tests/test_renderer_camera.py
git commit -m "feat(renderer): third-person chase camera with heading lerp"
```
