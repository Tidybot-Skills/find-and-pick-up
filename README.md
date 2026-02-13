# find-and-pick-up

Find and pick up a target object by chaining center-object and pick-up-object.

## What it does

1. **Find**: Rotates base ±20° to search for object if not visible
2. **Center**: Moves base to center object in wrist camera
3. **Pick**: Visual servo approach with gripper offset correction, then grasp

## Usage

```python
from find_and_pick_up import find_and_pick_up

# Find and pick up a banana
success = find_and_pick_up(target="banana")
```

## Dependencies

- `center-object`: Centers object in wrist camera using base movement
- `pick-up-object`: IBVS visual servoing pick with gripper offset correction

## Parameters

- `target`: YOLO class name to find and pick up (default: "banana")
- `verbose`: Print progress messages (default: True)

## Returns

- `True` if object was successfully grasped
- `False` if object not found or grasp failed
