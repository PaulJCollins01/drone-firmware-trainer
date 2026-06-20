# Minecraft Ground Plane — Design Spec

**Date:** 2026-06-20
**Status:** Approved

## Goal

Draw a flat Minecraft grass-coloured surface at Z=0 beneath the drone arena so the world feels grounded.

## Coordinate System

Z is the vertical axis. Z=0 is the ground floor (gravity pulls downward on Z; drone spawns at Z=1). The user confirmed Z=0 is the correct plane.

## Scope

Only `sim/renderer.py` is modified. No world geometry, physics, sensors, harness, or tests change.

## Design

### New color constant

```python
GRASS = (89, 125, 39)   # Minecraft grass green
```

Added to the color block alongside the existing Minecraft palette constants.

### Ground plane rendering

In `Renderer.draw()`, before the arena wireframe is drawn, project the 4 corners of the Z=0 floor using the current frame's view matrix:

```
corners (world space):
  (0,       0,       0)
  (ARENA_W, 0,       0)
  (ARENA_W, ARENA_H, 0)
  (0,       ARENA_H, 0)
```

Where `ARENA_W = 20.0` and `ARENA_H = 15.0` (from `sim/world.py`).

Project each corner via `_project(x, y, z, view)`. If all 4 corners return valid screen coordinates (none `None`), call:

```python
pygame.draw.polygon(self.screen, GRASS, [p0, p1, p2, p3])
```

**Draw order:** ground plane is drawn before the arena wireframe, obstacles, rays, and drone dot — so all other elements render on top of it correctly.

**Clipping:** If any corner projects to `None` (behind the camera or clipped), skip the polygon draw for that frame. The ground disappears momentarily in extreme camera angles — acceptable behaviour.

## What Does Not Change

- World geometry, physics, sensors, harness, scoring, firmware
- All existing tests
- All other renderer elements (arena wireframe, obstacles, goal, rays, drone dot, HUD)
