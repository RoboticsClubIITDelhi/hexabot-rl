import pybullet as p
import pybullet_data
import numpy as np
import math
import time
from serial import Serial


try:
    ser = Serial('COM9', 230400, timeout=0.01)
    print("Connected to ESP32 at 230400 Baud")
except Exception as e:
    print(f"Error: {e}. Is the Arduino Serial Monitor still open?")
    exit()

# ==============================================================================
# CONSTANTS & SETUP
# ==============================================================================
A, B, C = 0.074, 0.049975, 0.09552   
BODY_HEIGHT   = 0.10
NEUTRAL_REACH = 0.120
STEP_LENGTH   = 0.060
TURN_SWEEP    = 0.050  # Distance the foot sweeps sideways during a turn
STEP_HEIGHT   = 0.050
JOINT_FORCE   = 5.0
MAX_VEL       = 5.0

ALPHA_NEUTRAL_offset = 0.31552658847695203
GAMMA_NEUTRAL_offset = 1.2394829724774472
# Hardware servo tuning
# AX-12 at 300° range → 1024 steps → 3.413 steps/degree
# AX-12 at 360° range → 1024 steps → 2.844 steps/degree
STEPS_PER_DEGREE = 3.413

# Servo IDs on the physical robot [coxa, femur, tibia]
HARDWARE_IDS = {
    1: [23, 13, 11], 2: [19, 14, 16], 3: [5,  10, 17],
    4: [6,  12,  7], 5: [22, 18, 15], 6: [2,   4,  1],
}

HW_SIGNS = {
    1: {'coxa':  1, 'femur': 1, 'tibia': 1},
    2: {'coxa':  -1, 'femur':  1, 'tibia': 1},
    3: {'coxa': -1, 'femur':  1, 'tibia': 1},
    4: {'coxa': -1, 'femur':  1, 'tibia': 1},
    5: {'coxa': 1, 'femur': 1, 'tibia': 1},
    6: {'coxa':  1, 'femur': 1, 'tibia': 1},
}

JOINT_SIGNS = {
    1: {'coxa':  1, 'femur': -1, 'tibia':  1},
    2: {'coxa': -1, 'femur': -1, 'tibia':  1}, 
    3: {'coxa': -1, 'femur': -1, 'tibia':  1},
    4: {'coxa': -1, 'femur': -1, 'tibia':  1},
    5: {'coxa':  1, 'femur': -1, 'tibia':  1},
    6: {'coxa':  1, 'femur': -1, 'tibia':  1},
}

LEG_CONFIG = {
    1: ("L1_1",        "Revolute 102", "Revolute 100",  60.0),
    2: ("Revolute 63", "Revolute 73",  "Revolute 90",  120.0),
    3: ("Revolute 62", "Revolute 72",  "Revolute 97",  180.0),
    4: ("Revolute 61", "Revolute 71",  "Revolute 95", -120.0),
    5: ("Revolute 60", "Revolute 70",  "Revolute 80",  -60.0),
    6: ("Revolute 59", "Revolute 69",  "Revolute 92",    0.0),
}
def send_to_hardware(all_leg_angles):
    """
    Package and send IK angles to the physical robot over serial.

    Servo zero position is 512 (middle of 0-1023 range).
    We send (angle_deg * STEPS_PER_DEGREE) as offset from 512.
    """
    packet_data = []

    for leg_id, (lam, alpha, gamma) in all_leg_angles.items():
        hs = HW_SIGNS[leg_id]

        # Same delta-from-neutral logic as the simulation
        coxa_rad  =  lam   * hs['coxa']
        femur_rad = (alpha - ALPHA_NEUTRAL_offset) * hs['femur']
        tibia_rad = (gamma - GAMMA_NEUTRAL_offset) * hs['tibia']

        for servo_id, target_rad in zip(HARDWARE_IDS[leg_id],
                                        [coxa_rad, femur_rad, tibia_rad]):
            angle_deg   = math.degrees(target_rad)
            target_step = int(round(512 + angle_deg * STEPS_PER_DEGREE))
            target_step = max(0, min(1023, target_step))

            if servo_id == 1:          # debug print for one servo
                print(f"  servo {servo_id}: {angle_deg:.1f}° → step {target_step}")

            packet_data.append(str(servo_id))
            packet_data.append(str(target_step))

    motor_count = len(packet_data) // 2
    if motor_count > 0:
        packet_data.insert(0, str(motor_count))
        packet_string = f"<{','.join(packet_data)}>\n"
        try:
            ser.write(packet_string.encode())
        except Exception:
            pass

# ==============================================================================
# INVERSE KINEMATICS & CONTROL
# ==============================================================================
def IK(tx, ty, tz):
    lam = math.atan2(-tz, tx)
    reach = math.sqrt(tx**2 + tz**2) - A
    dist2 = reach**2 + ty**2
    dist = math.sqrt(dist2)

    cos_gamma = (B**2 + C**2 - dist2) / (2.0 * B * C)
    if cos_gamma > 1.0 or cos_gamma < -1.0:
        return None
    gamma = math.acos(cos_gamma)

    cos_alpha_base = (B**2 + dist2 - C**2) / (2.0 * B * dist)
    if cos_alpha_base > 1.0 or cos_alpha_base < -1.0:
        return None
        
    alpha_elev = math.atan2(ty, reach)     
    alpha_base = math.acos(cos_alpha_base)
    alpha = alpha_elev + alpha_base       

    return lam, alpha, gamma    

def set_joints(robot_id, leg_id, legs_dict, lam, alpha, gamma):
    s = JOINT_SIGNS[leg_id]
    coxa_target  = lam * s['coxa']
    femur_target = (alpha - ALPHA_NEUTRAL_offset) * s['femur']
    tibia_target = (gamma - GAMMA_NEUTRAL_offset) * s['tibia']

    indices = legs_dict[leg_id]["indices"]
    for idx, target in zip(indices, [coxa_target, femur_target, tibia_target]):
        p.setJointMotorControl2(
            robot_id, idx, p.POSITION_CONTROL,
            targetPosition=target, force=JOINT_FORCE, maxVelocity=MAX_VEL
        )

# ==============================================================================
# ARRAY GENERATORS (WALK AND TURN)
# ==============================================================================
def gen_walk_bezier_points(step_length, step_height, N):
    bz_pts = {}
    for lid, info in LEG_CONFIG.items():
        bz_pts[lid] = []
        mount_rad = math.radians(info[3])
        cos_m, sin_m = math.cos(mount_rad), math.sin(mount_rad)

        P0 = np.array([NEUTRAL_REACH + (-step_length / 2) * cos_m, -BODY_HEIGHT, -(-step_length / 2) * sin_m])
        P1 = np.array([NEUTRAL_REACH, -BODY_HEIGHT + step_height * 2.0, 0.0])
        P2 = np.array([NEUTRAL_REACH + (step_length / 2) * cos_m, -BODY_HEIGHT, -(step_length / 2) * sin_m])

        for t_idx in range(N):
            t = t_idx / float(N - 1)
            B_t = (1 - t)**2 * P0 + 2 * (1 - t) * t * P1 + t**2 * P2
            bz_pts[lid].append((B_t[0], B_t[1], B_t[2]))
    return bz_pts

def gen_walk_linear_points(step_length, step_height, N):
    lin_pts = {}
    for lid, info in LEG_CONFIG.items():
        lin_pts[lid] = []
        mount_rad = math.radians(info[3])
        cos_m, sin_m = math.cos(mount_rad), math.sin(mount_rad)

        for t_idx in range(N):
            t = t_idx / float(N - 1)
            fwd = step_length * (t - 0.5)
            tx  = NEUTRAL_REACH + fwd * cos_m
            ty  = -BODY_HEIGHT
            tz  = -fwd * sin_m
            lin_pts[lid].append((tx, ty, tz))
    return lin_pts

def gen_turn_bezier_points(turn_sweep, step_height, N):
    bz_pts = {}
    for lid in LEG_CONFIG.keys():
        bz_pts[lid] = []
        # Turn arcs ignore mount angles and sweep strictly left/right (tz axis)
        P0 = np.array([NEUTRAL_REACH, -BODY_HEIGHT, -turn_sweep / 2])
        P1 = np.array([NEUTRAL_REACH, -BODY_HEIGHT + step_height * 2.0, 0.0])
        P2 = np.array([NEUTRAL_REACH, -BODY_HEIGHT, turn_sweep / 2])

        for t_idx in range(N):
            t = t_idx / float(N - 1)
            B_t = (1 - t)**2 * P0 + 2 * (1 - t) * t * P1 + t**2 * P2
            bz_pts[lid].append((B_t[0], B_t[1], B_t[2]))
    return bz_pts

def gen_turn_linear_points(turn_sweep, step_height, N):
    lin_pts = {}
    for lid in LEG_CONFIG.keys():
        lin_pts[lid] = []
        for t_idx in range(N):
            t = t_idx / float(N - 1)
            sweep = turn_sweep * (t - 0.5)
            tx  = NEUTRAL_REACH
            ty  = -BODY_HEIGHT
            tz  = sweep 
            lin_pts[lid].append((tx, ty, tz))
    return lin_pts

def setup_robot(robot_id):
    j_name_to_id = {p.getJointInfo(robot_id, i)[1].decode(): i for i in range(p.getNumJoints(robot_id))}
    legs = {}
    for lid, (c, f, t, _ang) in LEG_CONFIG.items():
        legs[lid] = {"indices": (j_name_to_id[c], j_name_to_id[f], j_name_to_id[t])}
    return legs

# ==============================================================================
# MAIN SIMULATION LOOP
# ==============================================================================
def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    try:
        robot_id = p.loadURDF("robot.urdf", [0, 0, BODY_HEIGHT + 0.01])
    except Exception:
        print("Error: Ensure robot.urdf is in the same directory.")
        return

    legs_dict = setup_robot(robot_id)
    
    # Friction is critical for turning so feet grip the floor and twist the body
    p.changeDynamics(robot_id, -1, lateralFriction=2.0)
    for i in range(p.getNumJoints(robot_id)):
        p.changeDynamics(robot_id, i, lateralFriction=2.0)

    # 1. Pre-calculate ALL arrays
    N = 60
    walk_bz  = gen_walk_bezier_points(STEP_LENGTH, STEP_HEIGHT, N)
    walk_lin = gen_walk_linear_points(STEP_LENGTH, STEP_HEIGHT, N)
    turn_bz  = gen_turn_bezier_points(TURN_SWEEP, STEP_HEIGHT, N)
    turn_lin = gen_turn_linear_points(TURN_SWEEP, STEP_HEIGHT, N)

    print("Standing up...")
    lam0, alpha0, gamma0 = IK(NEUTRAL_REACH, -BODY_HEIGHT, 0.0)
    for _ in range(200):
        for leg_id in range(1, 7):
            set_joints(robot_id, leg_id, legs_dict, lam0, alpha0, gamma0)
        p.stepSimulation()
        time.sleep(1 / 240.0)

    group_a = [1, 3, 5]
    group_b = [2, 4, 6]
    t_idx = 0

    print("\n===============================")
    print(" READY! YOU MUST CLICK INSIDE THE PYBULLET WINDOW FIRST!")
    print(" [W] Forward   [S] Backward")
    print(" [A] Spin CCW  [D] Spin CW")
    print("===============================\n")

    while True:
        keys = p.getKeyboardEvents()
        
        w = keys.get(ord('w'), 0) & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED)
        s = keys.get(ord('s'), 0) & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED)
        a = keys.get(ord('a'), 0) & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED)
        d = keys.get(ord('d'), 0) & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED)

        if w or s or a or d:
            
            is_turning = a or d
            active_bz  = turn_bz if is_turning else walk_bz
            active_lin = turn_lin if is_turning else walk_lin
            frame_angles = {}
            for leg_id in range(1, 7):
                is_right_leg = leg_id in [1, 5, 6]
                
                # The Matrix: Determines if array is read Forward (1) or Backward (-1)
                if w:
                    dir_mult = 1
                elif s:
                    dir_mult = -1
                elif a:
                    dir_mult = -1 if is_right_leg else 1
                elif d:
                    dir_mult = 1 if is_right_leg else -1

                is_swinging = (leg_id in group_a)
                
                if is_swinging:
                    # Swing array normal read direction is t_idx
                    idx = t_idx if dir_mult == 1 else (N - 1 - t_idx)
                    pt = active_bz[leg_id][idx]
                else:
                    # Stance array normal read direction is inverted to push body
                    idx = (N - 1 - t_idx) if dir_mult == 1 else t_idx
                    pt = active_lin[leg_id][idx]

                angles = IK(*pt)
                if angles:
                    set_joints(robot_id, leg_id, legs_dict, *angles)
                    frame_angles[leg_id] = angles
            send_to_hardware(frame_angles)

            
            # Global clock ALWAYS ticks forward cleanly
            t_idx += 1
            if t_idx >= N:
                t_idx = 0
                group_a, group_b = group_b, group_a 

        p.stepSimulation()
        time.sleep(1 / 240.0)

if __name__ == "__main__":
    main()