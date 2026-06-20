# Third-Person Chase Camera — Design Spec

**Date:** 2026-06-20
**Status:** Approved

## Goal

Replace the static fixed camera in `sim/renderer.py` with a smoothed third-person chase cam that sits above and behind the drone relative to its movement direction, like a video game chase camera.

## Scope

Changes are confined to `sim/renderer.py`. No other files are touched.

## Camera Behaviour

- The camera sits `DIST = 6.0m` behind the drone along its heading and `Z_OFFSET = 3.0m` above it.
- The camera looks at a point `1.0m` ahead of the drone (not at the drone itself), so the drone sits in the lower-center of the frame.
- "Behind" is determined by the drone's XY velocity heading, smoothed with lerp so rapid firmware corrections don't spin the camera.
- When the drone is stationary (speed < 0.1 m/s), the camera holds the last known heading rather than snapping or spinning.

## Implementation

### State removed
- `_CAM_POS`, `_CAM_TARGET`, `_CAM_UP`, `_FOV_DEG`, `_VIEW` module-level constants are removed.
- `_F` and `_ASPECT` remain (projection constants, unchanged).

### State added to `Renderer.__init__`
- `self._cam_dir: np.ndarray` — current smoothed heading unit vector, initialized to `np.array([1.0, 0.0, 0.0])`.

### Constants added (module level)
| Name | Value | Purpose |
|---|---|---|
| `CAM_DIST` | `6.0` | metres behind drone |
| `CAM_Z_OFFSET` | `3.0` | metres above drone |
| `CAM_LOOK_AHEAD` | `1.0` | metres ahead of drone the camera targets |
| `CAM_ALPHA` | `0.05` | lerp weight per frame toward desired heading |
| `CAM_SPEED_THRESH` | `0.1` | m/s below which heading is frozen |
| `FOV_DEG` | `55.0` | unchanged from current value |

### Per-frame logic in `draw()`

```
speed_xy = sqrt(vx² + vy²)
if speed_xy > CAM_SPEED_THRESH:
    desired = normalize((vx, vy, 0))
    self._cam_dir = normalize((1-α)*self._cam_dir + α*desired)

eye    = drone_pos - self._cam_dir * CAM_DIST + (0, 0, CAM_Z_OFFSET)
target = drone_pos + self._cam_dir * CAM_LOOK_AHEAD
view   = _build_view(eye, target, up=(0,0,1))
```

`_project` is updated to accept `view` as a parameter instead of using the module-level `_VIEW`.

### Functions updated
- `_project(x, y, z, view)` — accepts view matrix as argument.
- `_draw_line(surf, color, a3, b3, view, width)` — passes view through to `_project`.
- `Renderer.draw()` — rebuilds view each frame, passes it to `_draw_line` and `_project` calls.

## What Does Not Change

- `_build_view` math
- `_F`, `_ASPECT` projection constants
- `_box_edges`
- All world, obstacle, goal, ray, and drone drawing logic
- HUD
- `hold()`, `close()`

## Success Criteria

- Camera follows behind the drone as it moves, rotating smoothly with heading changes.
- No spinning when the drone hovers in place.
- No changes to sweep output or test behaviour (camera is render-only).
