# Minecraft Overworld Day Colors Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the 7 color constants in `sim/renderer.py` with a Minecraft overworld day palette.

**Architecture:** Single-file constant swap — no logic changes. The color names stay the same; only their RGB values change.

**Tech Stack:** Python 3.9, pygame 2.5.2

## Global Constraints

- Only `sim/renderer.py` is modified — no other files change.
- Python 3.9 compatible.
- All existing tests must continue to pass.

---

### Task 1: Apply Minecraft Overworld Day Palette

**Files:**
- Modify: `sim/renderer.py:15-21`
- Test: `tests/test_renderer_camera.py` (existing — must still pass)

**Interfaces:**
- Consumes: existing color constants `WHITE`, `DIM`, `GREY`, `GREEN`, `BLUE`, `RED`, `BLACK` in `sim/renderer.py`
- Produces: same names, new RGB values

- [ ] **Step 1: Write the failing test**

Add to `tests/test_renderer_camera.py`:

```python
from sim.renderer import WHITE, DIM, GREY, GREEN, BLUE, RED, BLACK

def test_minecraft_overworld_palette():
    assert WHITE == (120, 167, 210), f"Expected sky blue, got {WHITE}"
    assert DIM   == (74,  74,  74),  f"Expected bedrock dark, got {DIM}"
    assert GREY  == (123, 123, 123), f"Expected stone grey, got {GREY}"
    assert GREEN == (248, 204, 38),  f"Expected gold beacon, got {GREEN}"
    assert BLUE  == (64,  164, 223), f"Expected water blue, got {BLUE}"
    assert RED   == (220, 70,  20),  f"Expected redstone orange, got {RED}"
    assert BLACK == (255, 255, 255), f"Expected MC UI white, got {BLACK}"
```

- [ ] **Step 2: Run test to verify it fails**

```
.venv/bin/python -m pytest tests/test_renderer_camera.py::test_minecraft_overworld_palette -v
```

Expected: FAIL — assertion errors showing the old RGB values.

- [ ] **Step 3: Replace color constants in `sim/renderer.py`**

Find the color block (lines 15–21) and replace with:

```python
BLACK  = (255, 255, 255)   # MC UI font white
WHITE  = (120, 167, 210)   # clear day sky
RED    = (220,  70,  20)   # redstone torch
GREEN  = (248, 204,  38)   # gold block beacon
BLUE   = ( 64, 164, 223)   # water/ice
GREY   = (123, 123, 123)   # stone block
DIM    = ( 74,  74,  74)   # bedrock/void edge
```

- [ ] **Step 4: Run all tests to verify they pass**

```
.venv/bin/python -m pytest tests/ -v
```

Expected: all tests PASS (including the 6 existing camera tests).

- [ ] **Step 5: Run smoke test**

```
PYTHONPATH=. .venv/bin/python tests/test_smoke.py
```

Expected: `SMOKE OK`

- [ ] **Step 6: Commit**

```bash
git add sim/renderer.py tests/test_renderer_camera.py
git commit -m "feat(renderer): Minecraft overworld day color palette"
```
