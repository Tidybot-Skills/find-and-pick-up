"""
find-and-pick-up skill
Chains center-object and pick-up-object to find and grasp a target object.

Dependencies:
  - center-object: Centers object in wrist camera using base movement
  - pick-up-object: Visual servoing pick with gripper offset correction
"""

from robot_sdk import arm, base, gripper, sensors, yolo, display
from robot_sdk.arm import ArmError
import numpy as np
import math
import time

# ============================================================================
# IMPORTED: center-object
# ============================================================================

def center_object(
    target="banana",
    tolerance=30,
    max_iterations=30,
    gain=0.00225,
    camera_id="309622300814",
    verbose=True
):
    """Center a target object in the wrist camera by moving the base."""
    CENTER_U, CENTER_V = 320, 240
    MAX_STEP = 0.04
    DAMPING = 0.5
    def log(msg):
        if verbose:
            print(msg)
    
    def detect_target():
        result = yolo.segment_camera(target, camera_id=camera_id)
        for det in result.detections:
            if det.class_name.lower() == target.lower():
                return det
        return None
    
    def rotational_center(det):
        """After finding object, rotate more to center it horizontally (U axis)"""
        U_TOLERANCE = 80
        FINE_ANGLE = math.radians(10)
        MAX_FINE_ROTATIONS = 6
        WIGGLE_STEP = 0.02  # 2cm
        
        for i in range(MAX_FINE_ROTATIONS):
            x1, y1, x2, y2 = det.bbox
            u = (x1 + x2) / 2
            u_err = u - CENTER_U
            
            if abs(u_err) < U_TOLERANCE:
                log(f"[center] Rotationally centered (u_err={u_err:.0f})")
                return det
            
            if u_err > 0:
                log(f"[center] Object right of center (u_err={u_err:.0f}), rotating -10°")
                base.move_delta(dtheta=-FINE_ANGLE)
            else:
                log(f"[center] Object left of center (u_err={u_err:.0f}), rotating +10°")
                base.move_delta(dtheta=FINE_ANGLE)
            
            time.sleep(0.3)
            det = detect_target()
            
            # Wiggle recovery if lost detection
            if det is None:
                log(f"[center] Lost detection, wiggling to recover...")
                for w in range(4):
                    dx = WIGGLE_STEP * (1 if w % 2 == 0 else -1)
                    dy = WIGGLE_STEP * (1 if (w // 2) % 2 == 0 else -1)
                    base.move_delta(dx=dx, dy=dy)
                    time.sleep(0.3)
                    det = detect_target()
                    if det:
                        log(f"[center] Recovered detection after wiggle")
                        break
                if det is None:
                    log(f"[center] Lost detection during rotational centering")
                    return None
        
        log(f"[center] Rotational centering done (max steps)")
        return det
    
    def search_rotate():
        """Rotate base ±30° then ±60° to search, then rotationally center"""
        angle_30 = math.radians(30)
        
        log(f"[center] Searching: rotating +30°...")
        base.move_delta(dtheta=angle_30)
        time.sleep(0.3)
        det = detect_target()
        if det:
            log(f"[center] Found at +30°, centering rotationally...")
            return rotational_center(det)
        
        log(f"[center] Searching: rotating -60° (to -30°)...")
        base.move_delta(dtheta=-2*angle_30)
        time.sleep(0.3)
        det = detect_target()
        if det:
            log(f"[center] Found at -30°, centering rotationally...")
            return rotational_center(det)
        
        base.move_delta(dtheta=angle_30)
        time.sleep(0.2)
        
        angle_60 = math.radians(60)
        
        log(f"[center] Searching: rotating +60°...")
        base.move_delta(dtheta=angle_60)
        time.sleep(0.3)
        det = detect_target()
        if det:
            log(f"[center] Found at +60°, centering rotationally...")
            return rotational_center(det)
        
        log(f"[center] Searching: rotating -120° (to -60°)...")
        base.move_delta(dtheta=-2*angle_60)
        time.sleep(0.3)
        det = detect_target()
        if det:
            log(f"[center] Found at -60°, centering rotationally...")
            return rotational_center(det)
        
        log(f"[center] Not found after ±30° and ±60° search")
        base.move_delta(dtheta=angle_60)
        time.sleep(0.3)
        return None
    
    log(f"[center] Starting centering for '{target}'")
    log(f"[center] Tolerance: {tolerance}px, Max iter: {max_iterations}")
    
    target_det = detect_target()
    
    if target_det is None:
        log(f"[center] Object not visible, starting rotation search...")
        target_det = search_rotate()
        if target_det is None:
            log(f"[center] FAILED - Object not found after search")
            return False, None
    
    consecutive_misses = 0
    wiggle_step = 0.02  # 2cm
    
    for iteration in range(max_iterations):
        result = yolo.segment_camera(target, camera_id=camera_id, confidence=0.15)
        
        target_det = None
        for det in result.detections:
            if det.class_name.lower() == target.lower():
                target_det = det
                break
        
        if target_det is None:
            consecutive_misses += 1
            log(f"[center] Iter {iteration}: No '{target}' detected ({consecutive_misses}/10)")
            
            if consecutive_misses >= 10:
                log(f"[center] FAILED - 10 consecutive detection failures")
                return False, None
            
            dx = wiggle_step * (1 if consecutive_misses % 2 == 0 else -1)
            dy = wiggle_step * (1 if (consecutive_misses // 2) % 2 == 0 else -1)
            log(f"[center] Wiggling base: dx={dx:.3f}m, dy={dy:.3f}m")
            base.move_delta(dx=dx, dy=dy)
            time.sleep(0.3)
            continue
        
        consecutive_misses = 0
        
        x1, y1, x2, y2 = target_det.bbox
        u, v = (x1 + x2) / 2, (y1 + y2) / 2
        u_err, v_err = u - CENTER_U, v - CENTER_V
        
        log(f"[center] Iter {iteration}: pos=({u:.0f}, {v:.0f}), err=({u_err:.0f}, {v_err:.0f}), conf={target_det.confidence:.2f}")
        
        if abs(u_err) < tolerance and abs(v_err) < tolerance:
            log(f"[center] SUCCESS - Object centered at ({u:.0f}, {v:.0f})")
            return True, (u, v)
        
        dx = max(-MAX_STEP, min(MAX_STEP, -gain * v_err * DAMPING))
        dy = max(-MAX_STEP, min(MAX_STEP, -gain * u_err * DAMPING))
        
        log(f"[center] Moving base: dx={dx:.4f}m, dy={dy:.4f}m")
        base.move_delta(dx=dx, dy=dy)
        time.sleep(0.3)
    
    log(f"[center] FAILED - Max iterations ({max_iterations}) reached")
    return False, None


# ============================================================================
# IMPORTED: pick-up-object (core functions)
# ============================================================================

# Configuration
TARGET_OBJECT = "banana"
CAMERA_ID = "309622300814"
DETECTION_CONFIDENCE = 0.15
GAIN_U_TO_DY = -0.0006
GAIN_V_TO_DX = -0.0006
GRIPPER_U_OFFSET = 0.0
GRIPPER_V_OFFSET = -120
OFFSET_START_Z = 0.0
OFFSET_END_Z = -0.25
PIXEL_TOLERANCE = 30
MAX_SERVO_ITERATIONS = 200
MAX_LATERAL_STEP_M = 0.05
MIN_LATERAL_STEP_M = 0.001
SERVO_MOVE_DURATION = 0.5
SEARCH_WIGGLE_ANGLE_DEG = 30
SEARCH_XY_STEP_M = 0.05
SEARCH_WIGGLE_DURATION = 0.4
MAX_SEARCH_FAILURES = 3
DESCEND_STEP_M = 0.05
DESCEND_PAUSE_PIXELS = 80
EE_FRAME_Z_THRESHOLD = -0.25  # Same as OFFSET_END_Z
EE_GAIN_U_TO_DY = +1.0 / 580
EE_GAIN_V_TO_DX = -1.0 / 660
GRASP_FORCE = 50
GRASP_SPEED = 200


def detect_object_2d(target, confidence=DETECTION_CONFIDENCE):
    result = yolo.segment_camera(target, camera_id=CAMERA_ID, confidence=confidence,
                                  save_visualization=True, mask_format="npz")
    detections = result.get_by_class(target) or result.detections
    if not detections:
        return None, None
    best = max(detections, key=lambda d: d.area if d.area > 0 else (d.bbox[2]-d.bbox[0])*(d.bbox[3]-d.bbox[1]))
    return best, result.image_shape


def get_object_pixel_center(detection):
    if detection.mask is not None:
        binary = (detection.mask > 0.5).astype(np.float32)
        if binary.sum() > 0:
            ys, xs = np.where(binary > 0)
            return float(xs.mean()), float(ys.mean())
    x1, y1, x2, y2 = detection.bbox
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def get_mask_orientation(detection):
    """Get object orientation angle from mask using PCA.
    
    Returns angle in radians for EE yaw rotation to align gripper PERPENDICULAR 
    to object's long axis (for grasping across the object).
    Clamped to ±90° to avoid joint limits.
    """
    if detection.mask is None:
        return 0.0
    
    mask = detection.mask
    binary = (mask > 0.5).astype(np.float32)
    ys, xs = np.where(binary > 0)
    
    if len(xs) < 10:
        return 0.0
    
    cx, cy = xs.mean(), ys.mean()
    xs_c, ys_c = xs - cx, ys - cy
    
    cov_xx = np.mean(xs_c * xs_c)
    cov_yy = np.mean(ys_c * ys_c)
    cov_xy = np.mean(xs_c * ys_c)
    
    theta = 0.5 * np.arctan2(2 * cov_xy, cov_xx - cov_yy)
    theta_perp = theta + math.pi / 2
    
    while theta_perp > math.pi:
        theta_perp -= 2 * math.pi
    while theta_perp < -math.pi:
        theta_perp += 2 * math.pi
    
    max_angle = math.pi / 2
    theta_perp = max(-max_angle, min(max_angle, theta_perp))
    
    return -theta_perp


def get_servo_target_pixel(image_shape, ee_z):
    h, w = image_shape[0], image_shape[1]
    if ee_z >= OFFSET_START_Z:
        ratio = 0.0
    elif ee_z <= OFFSET_END_Z:
        ratio = 1.0
    else:
        ratio = (OFFSET_START_Z - ee_z) / (OFFSET_START_Z - OFFSET_END_Z)
    return w / 2.0 + GRIPPER_U_OFFSET * ratio, h / 2.0 + GRIPPER_V_OFFSET * ratio


def search_wiggle(target):
    wiggle_rad = math.radians(SEARCH_WIGGLE_ANGLE_DEG)
    
    det, shape = detect_object_2d(target)
    if det:
        return det, shape, False
    
    print(f"    Wiggle search: rotating +{SEARCH_WIGGLE_ANGLE_DEG}°...")
    arm.move_delta(dyaw=wiggle_rad, frame="ee", duration=SEARCH_WIGGLE_DURATION)
    time.sleep(0.2)
    det, shape = detect_object_2d(target)
    if det:
        arm.move_delta(dyaw=-wiggle_rad, frame="ee", duration=SEARCH_WIGGLE_DURATION)
        return det, shape, True
    
    print(f"    Wiggle search: rotating to -{SEARCH_WIGGLE_ANGLE_DEG}°...")
    arm.move_delta(dyaw=-2*wiggle_rad, frame="ee", duration=SEARCH_WIGGLE_DURATION * 1.5)
    time.sleep(0.2)
    det, shape = detect_object_2d(target)
    if det:
        arm.move_delta(dyaw=wiggle_rad, frame="ee", duration=SEARCH_WIGGLE_DURATION)
        return det, shape, True
    
    arm.move_delta(dyaw=wiggle_rad, frame="ee", duration=SEARCH_WIGGLE_DURATION)
    return None, None, False


def servo_descend(target):
    ee_x, ee_y, ee_z = sensors.get_ee_position()
    consecutive_failures = 0
    aligned_to_object = False
    
    print(f"\n--- Servo-Descend: approaching '{target}' ---")
    display.show_text(f"Approaching {target}...")
    display.show_face("thinking")
    
    for i in range(MAX_SERVO_ITERATIONS):
        ee_x, ee_y, ee_z = sensors.get_ee_position()
        use_ee_frame = ee_z < EE_FRAME_Z_THRESHOLD
        
        # Align gripper with object orientation when crossing EE frame threshold
        if use_ee_frame and not aligned_to_object:
            det_for_align, _ = detect_object_2d(target)
            if det_for_align is not None:
                orient_angle = get_mask_orientation(det_for_align)
                if abs(orient_angle) > 0.05:
                    print(f"    Aligning gripper to object: rotating {math.degrees(orient_angle):.1f}°")
                    arm.move_delta(dyaw=orient_angle, frame="ee", duration=0.5)
                    time.sleep(0.3)
                else:
                    print(f"    Object orientation ~0°, no rotation needed")
            aligned_to_object = True
        
        det, shape = detect_object_2d(target)
        if det is None:
            det, shape, _ = search_wiggle(target)
            if det is None:
                consecutive_failures += 1
                if consecutive_failures >= MAX_SEARCH_FAILURES:
                    return False
                continue
        consecutive_failures = 0
        
        obj_u, obj_v = get_object_pixel_center(det)
        cx, cy = get_servo_target_pixel(shape, ee_z)
        u_err, v_err = obj_u - cx, obj_v - cy
        error_mag = np.sqrt(u_err**2 + v_err**2)
        
        frame_str = "EE" if use_ee_frame else "BASE"
        print(f"  Iter {i+1}: err=({u_err:.0f},{v_err:.0f}) |{error_mag:.0f}px| [{frame_str}] Z={ee_z:.3f}m")
        
        if use_ee_frame:
            dx_lat = EE_GAIN_V_TO_DX * v_err
            dy_lat = EE_GAIN_U_TO_DY * u_err
        else:
            dx_lat = GAIN_V_TO_DX * v_err
            dy_lat = GAIN_U_TO_DY * u_err
        
        lat_norm = np.sqrt(dx_lat**2 + dy_lat**2)
        if lat_norm > MAX_LATERAL_STEP_M:
            scale = MAX_LATERAL_STEP_M / lat_norm
            dx_lat, dy_lat = dx_lat * scale, dy_lat * scale
        
        dz = (DESCEND_STEP_M if use_ee_frame else -DESCEND_STEP_M) if error_mag < DESCEND_PAUSE_PIXELS else 0
        
        try:
            arm.move_delta(dx=dx_lat, dy=dy_lat, dz=dz, frame="ee" if use_ee_frame else "base", duration=SERVO_MOVE_DURATION)
        except ArmError as e:
            print(f"  FLOOR CONTACT: {e}")
            return True
        time.sleep(0.2)
    
    return True


def pick_up_object(target=TARGET_OBJECT):
    print(f"=== Pick Object: '{target}' ===\n")
    
    print("Phase 0: Initializing gripper...")
    display.show_text(f"Picking up {target}")
    gripper.activate()
    gripper.open()
    time.sleep(0.5)
    
    print("Tilting EE -20 deg pitch (camera down)...")
    arm.move_delta(dpitch=math.radians(-20), frame="ee", duration=1.0)
    time.sleep(0.3)
    
    print("\nPhase 1: Initial detection...")
    det, shape = detect_object_2d(target)
    if det is None:
        det, shape, _ = search_wiggle(target)
    if det is None:
        print("ERROR: Object not detected. Aborting.")
        display.show_face("sad")
        return False
    
    print("\nPhase 2: Servo-descend...")
    reached = servo_descend(target)
    
    print("\nPhase 3: Grasping...")
    display.show_text(f"Grasping {target}...")
    grasped = gripper.grasp(speed=GRASP_SPEED, force=GRASP_FORCE)
    time.sleep(0.5)
    
    if grasped:
        print("  Object grasped!")
        display.show_face("happy")
    else:
        print("  WARNING: Grasp uncertain.")
    
    print("\nPhase 4: Going home...")
    arm.go_home()
    time.sleep(0.5)
    
    print("\nPhase 5: Opening gripper...")
    gripper.open()
    
    return grasped


# ============================================================================
# MAIN: find-and-pick-up
# ============================================================================

def find_and_pick_up(target="banana", verbose=True):
    """
    Find and pick up a target object.
    
    Chains:
      1. center-object: Rotate to find object, center it in camera
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


# Entry point
if __name__ == "__main__":
    result = find_and_pick_up(target="banana")
    print(f"\nResult: {'SUCCESS' if result else 'FAILED'}")
