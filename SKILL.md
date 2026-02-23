---
name: tb-find-and-pick-up
description: Find and pick up a target object by searching, centering, and grasping. Use when (1) the object may not be visible yet and needs searching, (2) "find the X and pick it up", (3) a higher-level task needs autonomous object retrieval.
---

# Find and Pick Up

Chains `tb-center-object` (base search + centering) with `tb-pick-up-object` (IBVS visual servoing grasp).

## Pipeline

1. **Find**: Rotate base ±20° to search for object if not visible
2. **Center**: Move base to center object in wrist camera
3. **Pick**: Visual servo approach with gripper offset correction, then grasp

## Usage

```python
from main import find_and_pick_up
success = find_and_pick_up(target="banana")
```

## Parameters

- `target`: YOLO class name to find and pick up (default: "banana")
- `verbose`: Print progress messages (default: True)

## Returns

- `True` if object was successfully grasped
- `False` if object not found or grasp failed

## Dependencies

`tb-center-object`, `tb-pick-up-object` — see `scripts/deps.txt`. Bundle with `tidybot-bundle` before submission.
