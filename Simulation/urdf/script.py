import pybullet as p
import pybullet_data
import numpy as np
import math
import time
from serial import Serial

# ==============================================================================
# HARDWARE CONNECTION
# ==============================================================================
try:
    ser = Serial('COM9', 230400, timeout=0.01)
    print("Connected to ESP32 at 230400 Baud")
except Exception as e:
    print(f"Error: {e}. Is the Arduino Serial Monitor still open?")
    exit()

# ==============================================================================
# ROBOT PARAMETERS & CALIBRATION
# ==============================================================================
A, B, C = 0.074, 0.049975, 0.09552   # coxa, femur, tibia link lengths (metres)
BODY_HEIGHT   = 0.12  # Height of body above ground (metres) — CHANGE THIS to raise/lower robot
NEUTRAL_REACH = 0.110   # How far out the foot rests from the body centre (metres)
STEP_LENGTH   = 0.040
JOINT_FORCE   = 5.0
MAX_VEL       = 5.0
ALPHA_NEUTRAL_offset,GAMMA_NEUTRAL_offset=0.31552658847695203, 1.2394829724774472

# Hardware servo tuning
# AX-12 at 300° range → 1024 steps → 3.413 steps/degree
# AX-12 at 360° range → 1024 steps → 2.844 steps/degree
STEPS_PER_DEGREE = 3.413

# Servo IDs on the physical robot [coxa, femur, tibia]
HARDWARE_IDS = {
    1: [23, 13, 11], 2: [19, 14, 16], 3: [5,  10, 17],
    4: [6,  12,  7], 5: [22, 18, 15], 6: [2,   4,  1],
}

# Leg joint names in the URDF + mounting angle on the body (degrees)
LEG_CONFIG = {
    1: ("L1_1",        "Revolute 102", "Revolute 100",  60.0),
    2: ("Revolute 63", "Revolute 73",  "Revolute 90",  120.0),
    3: ("Revolute 62", "Revolute 72",  "Revolute 97",  180.0),
    4: ("Revolute 61", "Revolute 71",  "Revolute 95", -120.0),
    5: ("Revolute 60", "Revolute 70",  "Revolute 80",  -60.0),
    6: ("Revolute 59", "Revolute 69",  "Revolute 92",    0.0),
}

# Sign conventions for PyBullet simulation joints.
# Flip a sign here if a simulated joint moves the wrong way.
JOINT_SIGNS = {
    1: {'coxa':  1, 'femur': -1, 'tibia':  1},
    2: {'coxa':  -1, 'femur': -1, 'tibia': 1},
    3: {'coxa': -1, 'femur': -1, 'tibia': 1},
    4: {'coxa': -1, 'femur': -1, 'tibia': 1},
    5: {'coxa': 1, 'femur': -1, 'tibia':  1},
    6: {'coxa':  1, 'femur': -1, 'tibia':  1},
}

# Sign conventions for the physical hardware servos.
# Flip a sign here if a real servo spins the wrong way.
HW_SIGNS = {
    1: {'coxa':  1, 'femur': 1, 'tibia': 1},
    2: {'coxa':  1, 'femur':  1, 'tibia': 1},
    3: {'coxa': 1, 'femur':  1, 'tibia': 1},
    4: {'coxa': 1, 'femur':  1, 'tibia': 1},
    5: {'coxa': 1, 'femur': 1, 'tibia': 1},
    6: {'coxa':  1, 'femur': 1, 'tibia': 1},
}

# ==============================================================================
# KINEMATICS ENGINE
# ==============================================================================
def IK(tx, ty, tz):
    """
    3-DOF inverse kinematics for one leg.

    The leg coordinate frame has:
      +X  = forward/outward (away from body)
      +Y  = up
      +Z  = lateral

    tx, ty, tz : foot target position in the LEG frame.
                 ty is POSITIVE upward, so pass -BODY_HEIGHT to put
                 the foot on the ground below the body.

    Returns [lambda, alpha, gamma] in radians:
      lambda : coxa (yaw) angle
      alpha  : femur (pitch) angle
      gamma  : tibia (pitch) angle
    """
    # Coxa rotation (yaw) to point at the foot
    lam = math.atan2(-tz, tx)

    # Horizontal distance from femur pivot to foot
    reach = math.sqrt(tx**2 + tz**2) - A   # subtract coxa length

    # 2-link IK (femur B, tibia C) in the vertical plane
    dist2 = reach**2 + ty**2
    dist  = math.sqrt(dist2)

    # Clamp for numerical safety
    cos_gamma = (B**2 + C**2 - dist2) / (2.0 * B * C)
    gamma = math.acos(max(-1.0, min(1.0, cos_gamma)))

    # Alpha: angle of femur above/below horizontal
    cos_alpha_base = (B**2 + dist2 - C**2) / (2.0 * B * dist)
    alpha_base = math.acos(max(-1.0, min(1.0, cos_alpha_base)))
    alpha_elev = math.atan2(ty, reach)        # elevation to foot
    alpha = alpha_elev + alpha_base           # femur angle from horizontal

    return lam, alpha, gamma


# Compute neutral (standing) angles once so we can print them for reference
_lam0, ALPHA_NEUTRAL, GAMMA_NEUTRAL = IK(NEUTRAL_REACH, -BODY_HEIGHT, 0.0)
print(f"Neutral IK  →  alpha={math.degrees(ALPHA_NEUTRAL):.2f}°  "
      f"gamma={math.degrees(GAMMA_NEUTRAL):.2f}°")

# ==============================================================================
# JOINT CONTROL — SIMULATION
# ==============================================================================
def set_joints(robot_id, leg_id, legs_dict, lam, alpha, gamma):
    """
    Send IK angles to the PyBullet joints of one leg.

    The URDF joint zeros are defined at the neutral standing pose.
    So we command (angle - neutral_angle) as the joint target, with
    the per-leg sign applied.

    For the coxa (yaw) the neutral is 0, so we just send lam * sign.
    For femur/tibia we send the delta from their neutral values.
    """
    s = JOINT_SIGNS[leg_id]
    print(ALPHA_NEUTRAL,GAMMA_NEUTRAL)

    coxa_target  =  lam   * s['coxa']
    femur_target = (alpha - ALPHA_NEUTRAL_offset) * s['femur']
    tibia_target = (gamma - GAMMA_NEUTRAL_offset) * s['tibia']

    indices = legs_dict[leg_id]["indices"]
    for idx, target in zip(indices, [coxa_target, femur_target, tibia_target]):
        p.setJointMotorControl2(
            robot_id, idx,
            p.POSITION_CONTROL,
            targetPosition=target,
            force=JOINT_FORCE,
            maxVelocity=MAX_VEL,
        )

# ==============================================================================
# HARDWARE SENDER
# ==============================================================================
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
# TRAJECTORY GENERATORS
# ==============================================================================
def gen_bezier_points(step_length, step_height, N):
    """Swing-phase trajectory: quadratic Bézier arc above ground."""
    bz_pts = {}
    for lid, info in LEG_CONFIG.items():
        bz_pts[lid] = []
        mount_rad = math.radians(info[3])
        cos_m, sin_m = math.cos(mount_rad), math.sin(mount_rad)

        # Start, apex, end in body-relative foot space
        P0 = np.array([NEUTRAL_REACH + (-step_length / 2) * cos_m,
                       -BODY_HEIGHT,
                       -(-step_length / 2) * sin_m])
        P1 = np.array([NEUTRAL_REACH,
                       -BODY_HEIGHT + step_height * 2.0,
                       0.0])
        P2 = np.array([NEUTRAL_REACH + (step_length / 2) * cos_m,
                       -BODY_HEIGHT,
                       -(step_length / 2) * sin_m])

        for t_idx in range(N):
            t   = t_idx / float(N - 1)
            B_t = (1 - t)**2 * P0 + 2 * (1 - t) * t * P1 + t**2 * P2
            bz_pts[lid].append((B_t[0], B_t[1], B_t[2]))
    return bz_pts


def gen_linear_points(step_length, step_height, N):
    """Stance-phase trajectory: foot slides backward along the ground."""
    lin_pts = {}
    for lid, info in LEG_CONFIG.items():
        lin_pts[lid] = []
        mount_rad = math.radians(info[3])
        cos_m, sin_m = math.cos(mount_rad), math.sin(mount_rad)

        for t_idx in range(N):
            t   = t_idx / float(N - 1)
            fwd = step_length * (t - 0.5)
            tx  = NEUTRAL_REACH + fwd * cos_m
            ty  = -BODY_HEIGHT
            tz  = -fwd * sin_m
            lin_pts[lid].append((tx, ty, tz))
    return lin_pts

# ==============================================================================
# TRIPOD GAIT
# ==============================================================================
def tripod_walk(robot_id, legs_dict, step_length, step_height, num_cycles):
    group_a = [1, 3, 5]
    group_b = [2, 4, 6]

    N      = 60
    bz_pts  = gen_bezier_points(step_length, step_height, N)
    lin_pts = gen_linear_points(step_length, step_height, N)

    for cycle in range(num_cycles * 2):
        print(f"Half-cycle {cycle + 1}")
        for t in range(N):
            frame_angles = {}

            for leg_id in range(1, 7):
                if leg_id in group_a:
                    pt = bz_pts[leg_id][t]           # swinging
                else:
                    pt = lin_pts[leg_id][N - 1 - t]  # pushing back

                lam, alpha, gamma = IK(*pt)
                set_joints(robot_id, leg_id, legs_dict, lam, alpha, gamma)
                frame_angles[leg_id] = (lam, alpha, gamma)

            send_to_hardware(frame_angles)
            p.stepSimulation()
            time.sleep(1 / 240.0)

        group_a, group_b = group_b, group_a   # swap tripod groups

# ==============================================================================
# SETUP
# ==============================================================================
def setup_robot(robot_id):
    j_name_to_id = {
        p.getJointInfo(robot_id, i)[1].decode(): i
        for i in range(p.getNumJoints(robot_id))
    }
    legs = {}
    for lid, (c, f, t, _ang) in LEG_CONFIG.items():
        legs[lid] = {"indices": (j_name_to_id[c], j_name_to_id[f], j_name_to_id[t])}
    return legs

# ==============================================================================
# MAIN
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

    # ── Settle into neutral stance ──────────────────────────────────────────
    print(f"Standing up  (BODY_HEIGHT = {BODY_HEIGHT} m) ...")
    lam0, alpha0, gamma0 = IK(NEUTRAL_REACH, -BODY_HEIGHT, 0.0)

    for _ in range(200):           # more iterations gives servos time to settle
        frame_angles = {}
        for leg_id in range(1, 7):
            set_joints(robot_id, leg_id, legs_dict, lam0, alpha0, gamma0)
            frame_angles[leg_id] = (lam0, alpha0, gamma0)

        send_to_hardware(frame_angles)
        p.stepSimulation()
        time.sleep(1 / 240.0)

    print("Neutral stance reached.")

    # ── Walk (uncomment when ready) ─────────────────────────────────────────
    tripod_walk(
        robot_id=robot_id,
        legs_dict=legs_dict,
        step_length=STEP_LENGTH,
        step_height=0.030,
        num_cycles=60,
    )

    print("Simulation running — close the PyBullet window to quit.")
    while True:
        p.stepSimulation()
        time.sleep(1 / 240.0)


if __name__ == "__main__":
    main()