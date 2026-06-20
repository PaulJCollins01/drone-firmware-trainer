# Minecraft Overworld Day Colors — Design Spec

**Date:** 2026-06-20
**Status:** Approved

## Goal

Replace the flat primary color constants in `sim/renderer.py` with a Minecraft overworld day palette to give the sim a more natural, immersive look.

## Scope

Only the 7 color constants in `sim/renderer.py` change. No logic, structure, or other files are touched.

## Color Mapping

| Constant | Current RGB | New RGB | Minecraft Reference |
|---|---|---|---|
| `WHITE` (background) | `(240, 240, 240)` | `(120, 167, 210)` | Clear day sky |
| `DIM` (arena outline) | `(180, 180, 180)` | `(74, 74, 74)` | Bedrock/void edge |
| `GREY` (obstacles) | `(130, 130, 130)` | `(123, 123, 123)` | Stone block |
| `GREEN` (goal marker) | `(60, 200, 80)` | `(248, 204, 38)` | Gold block beacon |
| `BLUE` (sensor rays) | `(80, 140, 230)` | `(64, 164, 223)` | Water/ice |
| `RED` (drone dot) | `(220, 60, 60)` | `(220, 70, 20)` | Redstone torch |
| `BLACK` (HUD text) | `(0, 0, 0)` | `(255, 255, 255)` | Minecraft UI font |

## What Does Not Change

All logic, rendering structure, camera math, HUD layout, and all files other than `sim/renderer.py` are untouched.

## Success Criteria

- Renderer displays the Minecraft overworld day palette when run with `--render`
- All existing tests continue to pass
