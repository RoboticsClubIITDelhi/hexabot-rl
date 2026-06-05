import pybullet as p
import time
import pybullet_data
import math
import keyboard
from serial import Serial

# ── Serial / Bluetooth connection ─────────────────────────────────────────────
try:
    ser = Serial('COM7', 230400, timeout=0.01)
    print("Connected to ESP32 at 230400 baud")
except Exception as e:
    print(f"Serial error: {e}")
    exit()

# ── Leg → PyBullet joint-index tables (5-element: [coxa, femur, tibia, state, counter]) ──
EGS = {
    'L1': [1,  3,  5,  0, 0],
    'L2': [8,  10, 12, 0, 0],
    'L3': [15, 17, 19, 0, 0],
    'L4': [22, 24, 26, 0, 0],
    'L5': [30, 32, 34, 0, 0],
    'L6': [37, 39, 41, 0, 0]
}
LEGS = {
    'L1': [1,  3,  5,  0, 0],
    'L2': [8,  10, 12, 0, 0],
    'L3': [15, 17, 19, 0, 0],
    'L4': [22, 24, 26, 0, 0],
    'L5': [30, 32, 34, 0, 0],
    'L6': [37, 39, 41, 0, 0]
}

# ── Hardware servo IDs (physical robot) ───────────────────────────────────────
HW_IDS = {
    'L1': [2,  4,  1],
    'L2': [22, 18, 15],
    'L3': [6,  12,  7],
    'L4': [5,  10, 17],
    'L5': [19, 14, 16],
    'L6': [23, 13, 11],
}

HW_SIGNS = {
    'L1': [1, 1, 1],
    'L2': [1, 1, 1],
    'L3': [1, 1, 1],
    'L4': [1, 1, 1],
    'L5': [1, 1, 1],
    'L6': [1, 1, 1],
}

STEPS_PER_DEG = 3.413


# ── rotation() remaps LEGS so gait phase shifts direction ─────────────────────
def rotation(x):
    global LEGS
    sss = ['L1', 'L2', 'L3', 'L4', 'L5', 'L6']
    sss = sss[-(6 - x):] + sss[:x]
    TLEGS = {}
    rs = 1
    for pq in sss:
        TLEGS[pq] = EGS[f'L{rs}']
        rs += 1
    LEGS = TLEGS


def get_orig_leg(leg):
    """Reverse-map a (possibly rotated) leg name to the original hardware leg.

    After rotation(), 'L1' in LEGS may hold the joint IDs of what was
    physically 'L3'. This function finds the original leg name so that
    send_hardware() uses the correct servo IDs from HW_IDS.
    """
    cur = tuple(LEGS[leg][:3])
    for orig_leg, joints in EGS.items():
        if tuple(joints[:3]) == cur:
            return orig_leg
    return leg   # fallback (should not occur)


# ── Hardware output helpers ────────────────────────────────────────────────────
def angle_to_step(rad, sign=1):
    deg = math.degrees(rad * sign)
    step = int(round(512 + deg * STEPS_PER_DEG))
    return max(0, min(1023, step))


def send_hardware(frame_angles):
    """Build and send a single serial packet for all legs in frame_angles."""
    parts = []
    for leg, (l, m, n) in frame_angles.items():
        orig_leg = get_orig_leg(leg)
        ids   = HW_IDS[orig_leg]
        signs = HW_SIGNS[orig_leg]
        steps = [
            angle_to_step(l,   signs[0]),
            angle_to_step(m,   signs[1]),
            angle_to_step(-n,  signs[2]),
        ]
        for sid, stp in zip(ids, steps):
            parts.append(str(sid))
            parts.append(str(stp))
    count = len(parts) // 2
    packet = f"<{count},{','.join(parts)}>\n"
    try:
        ser.write(packet.encode())
    except Exception:
        pass


# ── PyBullet setup ─────────────────────────────────────────────────────────────
URDF_PATH = "robot.urdf"
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=0.1)
mu  = p.getQuaternionFromEuler([0, 0, 0])
bot = p.loadURDF(URDF_PATH, [0, 0, 0.30], useFixedBase=False, baseOrientation=mu)

for i in LEGS:
    for j in range(3):
        p.enableJointForceTorqueSensor(bot, LEGS[i][j], enableSensor=1)

max_force = 10
mode = p.POSITION_CONTROL
p.setGravity(0, 0, -10)

for i in LEGS:
    p.resetJointState(bot, LEGS[i][1], targetValue=130 * math.pi / 180)
    p.resetJointState(bot, LEGS[i][2], targetValue=100 * math.pi / 180)

# ── Kinematics constants ───────────────────────────────────────────────────────
maxdis = 0
maxlev = 30 * math.pi / 180
i = 0
a = 74.015
b = 49.064
c = (95.52 + 96.933) / 2
d = 218
x = 24.6  * math.pi / 180
y = 69.3  * math.pi / 180

x1 = a + b * math.sin(x + 130 * math.pi / 180) + c * math.cos(y + 30 * math.pi / 180)
y1 = b * math.cos(x + 130 * math.pi / 180)     - c * math.sin(y + 30 * math.pi / 180)
z1 = 0

R1_2    = (x1 - a)**2 + y1**2
phi_1   = math.atan2(y1, x1 - a)
theta_1 = x - y

x41 = x1 - maxdis / 2
z41 = 0  - math.sqrt(3) * maxdis / 2
y41 = y1 - (d / 4) * math.sin(maxlev)

x2_y2_z2 = {
    'L1': [
        (x1 - maxdis) * math.cos(maxlev) - (y1 - (d/2)*math.sin(maxlev)) * math.sin(maxlev),
        (x1 - maxdis) * math.sin(maxlev) + (y1 - (d/2)*math.sin(maxlev)) * math.cos(maxlev),
        0],
    'L2': [
        x41*(math.cos(maxlev)*0.25+0.75) - y41*0.5*math.sin(maxlev) - z41*(math.sqrt(3)/4)*(1-math.cos(maxlev)),
        x41*(math.sin(maxlev))*0.5       + y41*math.cos(maxlev)      + (z41*math.sqrt(3)/2)*math.sin(maxlev),
       -x41*(math.sqrt(3)/4)*(1-math.cos(maxlev)) - (y41*math.sqrt(3)/2)*math.sin(maxlev) + z41*(math.cos(maxlev)*0.75+0.25)],
    'L3': [
        x41*(math.cos(maxlev)*0.25+0.75) + y41*0.5*math.sin(maxlev) + z41*(math.sqrt(3)/4)*(1-math.cos(maxlev)),
       -x41*(math.sin(maxlev))*0.5       + y41*math.cos(maxlev)     + (z41*math.sqrt(3)/2)*math.sin(maxlev),
        x41*(math.sqrt(3)/4)*(1-math.cos(maxlev)) - (y41*math.sqrt(3)/2)*math.sin(maxlev) + z41*(math.cos(maxlev)*0.75+0.25)],
    'L4': [
        (x1 + maxdis) * math.cos(maxlev) + (y1 + (d/2)*math.sin(maxlev)) * math.sin(maxlev),
       -(x1 + maxdis) * math.sin(maxlev) + (y1 + (d/2)*math.sin(maxlev)) * math.cos(maxlev),
        0],
    'L5': [
        x41*(math.cos(maxlev)*0.25+0.75) + y41*0.5*math.sin(maxlev) + z41*(math.sqrt(3)/4)*(1-math.cos(maxlev)),
       -x41*(math.sin(maxlev))*0.5       + y41*math.cos(maxlev)     + (z41*math.sqrt(3)/2)*math.sin(maxlev),
       -(x41*(math.sqrt(3)/4)*(1-math.cos(maxlev)) - (y41*math.sqrt(3)/2)*math.sin(maxlev) + z41*(math.cos(maxlev)*0.75+0.25))],
    'L6': [
        x41*(math.cos(maxlev)*0.25+0.75) - y41*0.5*math.sin(maxlev) - z41*(math.sqrt(3)/4)*(1-math.cos(maxlev)),
        x41*(math.sin(maxlev))*0.5       + y41*math.cos(maxlev)     + (z41*math.sqrt(3)/2)*math.sin(maxlev),
       -(-x41*(math.sqrt(3)/4)*(1-math.cos(maxlev)) - (y41*math.sqrt(3)/2)*math.sin(maxlev) + z41*(math.cos(maxlev)*0.75+0.25))]
}

xrot   = 180
trot   = min(30, xrot)
beta_1 = math.atan2(b + c * math.sin(theta_1), c * math.cos(theta_1))


# ── Inverse kinematics ─────────────────────────────────────────────────────────
def ik(x2, y2, z2, leg, frame_angles):
    """Compute IK, drive PyBullet joints, and record angles for hardware output.

    Angles are only written to frame_angles when the leg is not in its
    state-machine hold (LEGS[leg][3]==1), so send_hardware only receives
    legs that are actively commanded this frame.
    """
    if LEGS[leg][3] == 1 and LEGS[leg][4] < 25:
        LEGS[leg][4] += 1
        return
    elif LEGS[leg][3] == 1 and LEGS[leg][4] == 25:
        LEGS[leg][3] = 0
        LEGS[leg][4] = 0
        return

    l = math.atan2(-z2, x2)
    R2_2 = (math.sqrt(x2**2 + z2**2) - a)**2 + y2**2
    r = (b**2 + c**2 - R2_2) / (2 * b * c)
    if r > 1:
        n = math.pi/2 + x - y
    elif r < -1:
        n = math.pi/2 + x - y - math.pi
    else:
        n = math.pi/2 + x - y - math.acos(r)

    q = (-c**2 + b**2 + R2_2) / (2 * b * math.sqrt(R2_2))
    if q > 1:
        sup = 0
    elif q < -1:
        sup = math.pi
    else:
        sup = math.acos(q)

    pu = (a**2 + R2_2 - x2**2 - y2**2 - z2**2) / (2 * a * math.sqrt(R2_2))
    if pu > 1:
        m = 3*math.pi/2 - x - sup
    elif pu < -1:
        m = 3*math.pi/2 - x - sup - math.pi
    else:
        m = 3*math.pi/2 - x - sup - math.acos(pu)

    p.setJointMotorControl2(bot, LEGS[leg][0], controlMode=mode,
                            targetPosition=l,  force=max_force, maxVelocity=1000)
    p.setJointMotorControl2(bot, LEGS[leg][1], controlMode=mode,
                            targetPosition=m,  force=max_force, maxVelocity=1000)
    p.setJointMotorControl2(bot, LEGS[leg][2], controlMode=mode,
                            targetPosition=-n, force=max_force, maxVelocity=1000)

    # Store angles for hardware packet (sign convention matches script 2)
    frame_angles[leg] = (-l, -m, n)


spd = 60


def bz(t, leg, frame_angles, rl=0):
    s = t / spd
    if rl == 0:
        x2 = x2_y2_z2[leg][0]
        y2 = x2_y2_z2[leg][1]
        z2 = x2_y2_z2[leg][2]
    else:
        x2 = x1 * math.cos(trot * math.pi / 180)
        y2 = y1
        z2 = x1 * math.sin(trot * math.pi / 180)*rl
    ik(x1 + (x2 - x1)*s, y1*(1-s)**2 + y2*s**2, z2*s, leg, frame_angles)


# ── Main loop ─────────────────────────────────────────────────────────────────
jk = 0
hw_started = False   # one-shot flag for the 3-second startup delay

while True:
    # ── Simulation warm-up (500 steps, no hardware output) ──
    if i < 500:
        p.stepSimulation()
        time.sleep(1. / 240.)
        i += 1
        continue

    # ── One-shot 3-second delay so the physical robot is ready ──
    if not hw_started:
        time.sleep(3)
        hw_started = True

    # ── Keyboard input ──
    lo = 0
    sx = 0
    rl = 0
    if keyboard.is_pressed('w'):
        lo += 1
    if keyboard.is_pressed('a'):
        lo += 3
    if keyboard.is_pressed('s'):
        lo -= 1
    if keyboard.is_pressed('d'):
        lo -= 3
    if keyboard.is_pressed('r'):
        rl += 1
    if keyboard.is_pressed('t'):
        rl -= 1

    if   lo ==  1:  sx = 0
    elif lo == -2:  sx = 1
    elif lo == -4:  sx = 2
    elif lo == -1:  sx = 3
    elif lo ==  2:  sx = 4
    elif lo ==  4:  sx = 5
    elif rl == 0:
        # ── Idle: hold standing position ──
        frame_angles = {}
        ik(x1, y1, 0, 'L1', frame_angles)
        ik(x1, y1, 0, 'L2', frame_angles)
        ik(x1, y1, 0, 'L3', frame_angles)
        ik(x1, y1, 0, 'L4', frame_angles)
        ik(x1, y1, 0, 'L5', frame_angles)
        ik(x1, y1, 0, 'L6', frame_angles)
        send_hardware(frame_angles)
        p.stepSimulation()
        time.sleep(1. / 240.)
        continue

    rotation(sx)
    print(sx)
    print(x1, y1)
    print(LEGS)

    theta = maxlev * (math.pi / 180) * math.sin(math.pi * (i - 500) / spd)
    dis   = maxdis * math.sin(math.pi * (i - 500) / spd)
    phase = (i - 500) % (2 * spd)

    # ── Walking gait ──────────────────────────────────────────────────────────
    if rl == 0:
        if phase < spd:
            dis = maxdis * math.sin(math.pi * (i + spd - 500) / spd)
            x2  = (x1 - dis)*math.cos(theta) - (y1 - (d/2)*math.sin(theta))*math.sin(theta)
            y2  = (x1 - dis)*math.sin(theta) + (y1 - (d/2)*math.sin(theta))*math.cos(theta)
            x42 = x1 + dis / 2
            z42 = -math.sqrt(3) * dis / 2
            y42 = y1 + (d / 4) * math.sin(theta)
            x5  = x42*(math.cos(theta)*0.25+0.75) + y42*0.5*math.sin(theta)  + z42*(math.sqrt(3)/4)*(1-math.cos(theta))
            y5  = -x42*(math.sin(theta))*0.5      + y42*math.cos(theta)       + (z42*math.sqrt(3)/2)*math.sin(theta)
            z5  = x42*(math.sqrt(3)/4)*(1-math.cos(theta)) - (y42*math.sqrt(3)/2)*math.sin(theta) + z42*(math.cos(theta)*0.75+0.25)
            frame_angles = {}
            ik(x2, y2,  0,  'L1', frame_angles)
            ik(x5, y5, -z5, 'L5', frame_angles)
            ik(x5, y5,  z5, 'L3', frame_angles)
            bz(phase + 1, 'L2', frame_angles)
            bz(phase + 1, 'L4', frame_angles)
            bz(phase + 1, 'L6', frame_angles)
            send_hardware(frame_angles)

        if phase >= spd:
            x3  = (x1 + dis)*math.cos(theta) + (y1 + (d/2)*math.sin(theta))*math.sin(theta)
            y3  = -(x1 + dis)*math.sin(theta) + (y1 + (d/2)*math.sin(theta))*math.cos(theta)
            x42 = x1 - dis / 2
            z42 = -math.sqrt(3) * dis / 2
            y42 = y1 - (d / 4) * math.sin(theta)
            x4  = x42*(math.cos(theta)*0.25+0.75) - y42*0.5*math.sin(theta)  - z42*(math.sqrt(3)/4)*(1-math.cos(theta))
            y4  = x42*(math.sin(theta))*0.5        + y42*math.cos(theta)       + (z42*math.sqrt(3)/2)*math.sin(theta)
            z4  = -x42*(math.sqrt(3)/4)*(1-math.cos(theta)) - (y42*math.sqrt(3)/2)*math.sin(theta) + z42*(math.cos(theta)*0.75+0.25)
            frame_angles = {}
            ik(x3, y3,  0,  'L4', frame_angles)
            ik(x4, y4,  z4, 'L2', frame_angles)
            ik(x4, y4, -z4, 'L6', frame_angles)
            bz(phase - spd + 1, 'L1', frame_angles)
            bz(phase - spd + 1, 'L3', frame_angles)
            bz(phase - spd + 1, 'L5', frame_angles)
            send_hardware(frame_angles)

        p.stepSimulation()
        time.sleep(1. / 240.)
        i += 1

    # ── Rotation gait ─────────────────────────────────────────────────────────
    else:
        rot = trot * math.pi / 180 * math.sin(math.pi * (i - 500) / spd) * rl
        if phase < spd:
            frame_angles = {}
            ik(x1*math.cos(rot), y1, -x1*math.sin(rot), 'L1', frame_angles)
            ik(x1*math.cos(rot), y1, -x1*math.sin(rot), 'L5', frame_angles)
            ik(x1*math.cos(rot), y1, -x1*math.sin(rot), 'L3', frame_angles)
            bz(phase + 1, 'L2', frame_angles, rl)
            bz(phase + 1, 'L4', frame_angles, rl)
            bz(phase + 1, 'L6', frame_angles, rl)
            send_hardware(frame_angles)

        if phase >= spd:
            rot = trot * math.pi / 180 * math.sin(math.pi * (i + spd - 500) / spd)
            frame_angles = {}
            ik(x1*math.cos(rot), y1, -x1*math.sin(rot), 'L2', frame_angles)
            ik(x1*math.cos(rot), y1, -x1*math.sin(rot), 'L4', frame_angles)
            ik(x1*math.cos(rot), y1, -x1*math.sin(rot), 'L6', frame_angles)
            bz(phase - spd + 1, 'L1', frame_angles, rl)
            bz(phase - spd + 1, 'L3', frame_angles, rl)
            bz(phase - spd + 1, 'L5', frame_angles, rl)
            send_hardware(frame_angles)

        p.stepSimulation()
        time.sleep(1. / 240.)
        i += 1
