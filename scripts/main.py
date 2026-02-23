"""
find-and-pick-up skill
Chains center-object and pick-up-object to find and grasp a target object.

Dependencies (resolved by tidybot-bundle):
  - center-object: Centers object in wrist camera using base movement
  - pick-up-object: Visual servoing pick with IBVS servo-descend

Usage:
  tidybot-bundle find-and-pick-up -o bundled.py
  # Then submit bundled.py via /code/execute
"""

from robot_sdk import display

# These functions are provided by dependencies (inlined by tidybot-bundle):
#   center_object()  — from center-object/main.py
#   pick_up_object()  — from pick-up-object/main.py


def find_and_pick_up(target="banana", verbose=True):
    """
    Find and pick up a target object.

    Chains:
      1. center-object: Rotate base to find object, center it in camera
      2. pick-up-object: Visual servo approach and grasp

    Args:
        target: Object class name to find and pick up
        verbose: Print progress messages

    Returns:
        bool: True if object was successfully grasped
    """
    print(f"=== Find and Pick Up: '{target}' ===\n")

    # Step 1: Find and center
    print("[STEP 1] Finding and centering object...")
    centered, pos = center_object(target=target, verbose=verbose)

    if not centered:
        print(f"FAILED: Could not find '{target}'")
        display.show_text(f"{target} not found!")
        display.show_face("sad")
        return False

    print(f"Object centered at {pos}")

    # Step 2: Pick it up
    print("\n[STEP 2] Picking up object...")
    success = pick_up_object(target=target)

    if success:
        print(f"\n=== Successfully found and picked '{target}'! ===")
        display.show_face("excited")
    else:
        print(f"\n=== Pick attempt complete (grasp uncertain) ===")

    return success


if __name__ == "__main__":
    result = find_and_pick_up(target="banana")
    print(f"\nResult: {'SUCCESS' if result else 'FAILED'}")
