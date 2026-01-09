# Bugfix - Initialization Order Error

## Issue

**Error Message:**
```
AttributeError: 'Renderer' object has no attribute 'gantry_width'
```

**Location:** `ui/renderer.py`, line 128 in `_draw_interference_zones()`

## Root Cause

The initialization order in `_setup_main_view()` was incorrect:

```python
# WRONG ORDER:
def _setup_main_view(self):
    ...
    self._draw_interference_zones()  # Called BEFORE gantry_width defined
    ...
    self.gantry_width = 150  # Defined AFTER being used
```

The `_draw_interference_zones()` method needs `self.gantry_width` to draw collision zones, but it was called before the variable was defined.

## Fix

Moved `gantry_width` definition before calling `_draw_interference_zones()`:

```python
# CORRECT ORDER:
def _setup_main_view(self):
    ...
    # Define gantry_width FIRST
    self.gantry_width = 150
    
    # Now can call methods that use it
    self._draw_interference_zones()
    ...
```

## Verification

```bash
cd weld_sim
python -c "
from ui.renderer import Renderer
from simulator.simulator import Simulator
# ... setup code ...
renderer = Renderer(sim, SCENE)
print('✅ Renderer created successfully!')
"
```

Expected output:
```
✅ Renderer created successfully!
  Gantry width: 150mm
  Robot range circles: 4
  Collision zones: 4
```

## Status

✅ **FIXED** in version 2.1.0_fixed

---

**Date:** January 9, 2026  
**Severity:** High (prevents execution)  
**Status:** ✅ RESOLVED
