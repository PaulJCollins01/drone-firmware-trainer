# Minecraft Ground Plane Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Draw a flat Minecraft grass-coloured filled polygon at Z=0 beneath the drone arena.

**Architecture:** Add a `GRASS` color constant and a `_draw_ground` helper to `sim/renderer.py`. `Renderer.draw()` calls `_draw_ground` before all other draw calls so the ground sits behind everything. If any corner projects behind the camera, the draw is skipped for that frame.

**Tech Stack:** Python 3.9, pygame 2.5.2, numpy

## Global Constraints

- Only `sim/renderer.py` and `tests/test_renderer_camera.py` are modified.
- Python 3.9 compatible (no `match`, no `X | Y` union syntax).
- All existing tests must continue to pass.
- Ground is drawn before the arena wireframe so all other elements render on top.

---

### Task 1: Add GRASS constant and ground plane drawing

**Files:**
- Modify: `sim/renderer.py`
- Test: `tests/test_renderer_camera.py`

**Interfaces:**
- Consumes: `_project(x, y, z, view: np.ndarray) -> Optional[Tuple[int, int]]` (existing), `pygame.draw.polygon` (pygame)
- Produces: `GRASS: tuple` constant, `_draw_ground(surf, view: np.ndarray, arena_w: float, arena_h: float) -> None`

- [ ] **Step 1: Write the failing tests**

Add to `tests/test_renderer_camera.py`:

```python
from sim.renderer import GRASS, _draw_ground

def test_grass_color():
    assert GRASS == (89, 125, 39), f"Expected Minecraft grass green, got {GRASS}"

def test_draw_ground_skips_when_corner_behind_camera():
    import numpy as np
    # A view matrix where all ground corners are behind the camera (z >= -0.1)
    # Use identity — points at z=0 project to p[2]=0 which is >= -0.1, so all return None
    view = np.eye(4)
    # Should not raise even when all corners clip
    _draw_ground(None, view, 20.0, 15.0)
```

- [ ] **Step 2: Run tests to verify they fail**

```
.venv/bin/python -m pytest tests/test_renderer_camera.py::test_grass_color tests/test_renderer_camera.py::test_draw_ground_skips_when_corner_behind_camera -v
```

Expected: FAIL — `ImportError: cannot import name 'GRASS'`

- [ ] **Step 3: Add GRASS constant to `sim/renderer.py`**

In the color block (after the existing Minecraft palette constants), add:

```python
GRASS  = (89,  125,  39)   # Minecraft grass green
```

- [ ] **Step 4: Add `_draw_ground` helper to `sim/renderer.py`**

Add this function after `_draw_line` and before `_box_edges`:

```python
def _draw_ground(surf, view: np.ndarray, arena_w: float, arena_h: float) -> None:
    corners_3d = [
        (0.0,     0.0,     0.0),
        (arena_w, 0.0,     0.0),
        (arena_w, arena_h, 0.0),
        (0.0,     arena_h, 0.0),
    ]
    pts = [_project(x, y, z, view) for x, y, z in corners_3d]
    if any(p is None for p in pts):
        return
    pygame.draw.polygon(surf, GRASS, pts)
```

- [ ] **Step 5: Call `_draw_ground` in `Renderer.draw()` before the arena wireframe**

In `Renderer.draw()`, locate the line `self.screen.fill(WHITE)`. Immediately after it (and after the view matrix is computed), add:

```python
_draw_ground(self.screen, view, self.world.arena_w, self.world.arena_h)
```

The full draw order in `draw()` must be:
1. `self.screen.fill(WHITE)` — sky background
2. `_draw_ground(...)` — grass plane (NEW — insert here)
3. Arena wireframe
4. Obstacles
5. Goal marker
6. Rays
7. Drone dot
8. HUD

- [ ] **Step 6: Run all tests**

```
.venv/bin/python -m pytest tests/ -v
```

Expected: all tests PASS (7 existing + 2 new = 9 total).

- [ ] **Step 7: Run smoke test**

```
PYTHONPATH=. .venv/bin/python tests/test_smoke.py
```

Expected: `SMOKE OK`

- [ ] **Step 8: Commit**

```bash
git add sim/renderer.py tests/test_renderer_camera.py
git commit -m "feat(renderer): Minecraft grass ground plane at Z=0"
```
