import pybullet as p
import time
import pybullet_data
import math
import serial

# --- 1. HARDWARE & BLUETOOTH SETUP ---
ENABLE_HARDWARE = False
ser = None
COM_PORT = 'COM18'  # Change this to match your OS deployment
BAUD_RATE = 230400

if ENABLE_HARDWARE:
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.01, write_timeout=0.01)
        print(f"Successfully connected to ESP32 via Bluetooth on {COM_PORT}")
    except Exception as e:
        print(f"Serial error: {e}. Running in SIMULATION ONLY mode.")
        ENABLE_HARDWARE = False

LEGS = {'L1': [1, 3, 5], 'L2': [8, 10, 12], 'L3': [15, 17, 19], 
        'L4': [22, 24, 26], 'L5': [30, 32, 34], 'L6': [37, 39, 41]}

# NOTE: Adjust these IDs to match your physical wiring layout
HW_IDS = {'L1': [2, 4, 1],   'L2': [22, 18, 15], 'L3': [6, 12, 7], 
          'L4': [5, 10, 17], 'L5': [19, 14, 16], 'L6': [23, 13, 11]}

# NOTE: Change 1 to -1 for any physical motor spinning backwards
HW_SIGNS = {'L1': [1, 1, 1],  'L2': [1, 1, 1],  'L3': [1, 1, 1], 
            'L4': [1, 1, 1],  'L5': [1, 1, 1],  'L6': [1, 1, 1]}

STEPS_PER_DEG = 3.413  # Map 0-1023 steps across standard physical servo ranges

def angle_to_step(rad, sign=1):
    deg = math.degrees(rad * sign)
    step = int(round(512 + deg * STEPS_PER_DEG))
    return max(0, min(1023, step))

def send_hardware(leg_angles):
    if not ENABLE_HARDWARE or ser is None: 
        return
    parts = []
    for leg, (l, m, n) in leg_angles.items():
        ids, signs = HW_IDS[leg], HW_SIGNS[leg]
        # Map simulated angles directly to calibrated hardware targets
        steps = [angle_to_step(l, signs[0]), angle_to_step(m, signs[1]), angle_to_step(-n, signs[2])]
        for sid, stp in zip(ids, steps): 
            parts.extend([str(sid), str(stp)])
            
    packet = f"<{len(parts)//2},{','.join(parts)}>\n"
    try: 
        ser.write(packet.encode())
    except serial.SerialTimeoutException: 
        pass  # Skip dropped frames gracefully to prevent visual jitter
    except Exception as e: 
        print(f"Bluetooth transmission error: {e}")

# --- 2. PYBULLET SETUP ---
print("Initializing PyBullet Environment...")
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=10.0)
bot = p.loadURDF("robot.urdf", [0, 0, 0.30], useFixedBase=False)
p.setGravity(0, 0, -10)
mode = p.POSITION_CONTROL

# --- 3. INVERSE KINEMATICS ENGINE & STANCES ---
a, b, c = 74.015, 49.064, 96.933
x_angle = 24.6 * math.pi / 180
y_angle = 68.2 * math.pi / 180

x1_high = a + b * math.sin(x_angle + 130 * math.pi / 180) + c * math.cos(y_angle + 30 * math.pi / 180)
y1_high = b * math.cos(x_angle + 130 * math.pi / 180) - c * math.sin(y_angle + 30 * math.pi / 180)

x1_low = 60
y1_low = y1_high + 60

def ik(x2, y2, z2, leg, frame_angles):
    l = math.atan2(-z2, x2)
    R2_2 = (math.sqrt(x2**2 + z2**2) - a)**2 + y2**2
    r = max(-1.0, min(1.0, (b**2 + c**2 - R2_2) / (2 * b * c)))
    n = math.pi / 2 + x_angle - y_angle - math.acos(r)
    q = max(-1.0, min(1.0, (-c**2 + b**2 + R2_2) / (2 * b * math.sqrt(R2_2))))
    m = 3 * math.pi / 2 - x_angle - math.acos(q) - math.acos(max(-1.0, min(1.0, (a**2 + R2_2 - x2**2 - y2**2 - z2**2) / (2 * a * math.sqrt(R2_2)))))
    
    for i, target in enumerate([l, m, -n]):
        p.setJointMotorControl2(bot, LEGS[leg][i], controlMode=mode, targetPosition=target, force=2.5)
    frame_angles[leg] = (-l, -m, n)

# --- 4. THE MACARENA DANCE FUNCTION ---
def dance_macarena():
    frames_per_beat = 280
    lunge_dist = 40 
    tap_height = 50       
    tap_extend = 100       
    
    leg_angles = {'L1': 0, 'L6': math.pi/3, 'L5': 2*math.pi/3, 'L4': math.pi, 'L3': -2*math.pi/3, 'L2': -math.pi/3}
    tap_sequence = ['L1', 'L2', 'L6', 'L3', 'L5', 'L4']
    
    total_frames = frames_per_beat * 9 
    print("Executing Choreography on Hardware...")
    
    for t in range(total_frames):
        current_beat = t // frames_per_beat
        beat_progress = (t % frames_per_beat) / frames_per_beat
        
        g_X = 0
        g_Z = 0
        
        if current_beat < 6:
            base_x = x1_low
            base_y = y1_low
            active_leg = tap_sequence[current_beat]
            active_gamma = leg_angles[active_leg]
            arc_multiplier = math.sin(math.pi * beat_progress)
            g_X = lunge_dist * math.cos(active_gamma) * arc_multiplier
            g_Z = lunge_dist * math.sin(active_gamma) * arc_multiplier
            
        elif current_beat == 4:
            interp = (1 - math.cos(math.pi * beat_progress)) / 2
            base_x = x1_low + (x1_high - x1_low) * interp
            base_y = y1_low + (y1_high - y1_low) * interp
            
        else:
            base_x = x1_high
            base_y = y1_high
            circle_progress = (t - (7 * frames_per_beat)) / (1 * frames_per_beat)
            circle_radius = 25
            g_X = circle_radius * math.sin(2 * math.pi * circle_progress)
            g_Z = -circle_radius * (math.cos(2 * math.pi * circle_progress) - 1)
            
        frame_angles = {}
        
        for leg_name, gamma in leg_angles.items():
            local_x = base_x - (g_X * math.cos(gamma) + g_Z * math.sin(gamma))
            local_y = base_y
            local_z = -(-g_X * math.sin(gamma) + g_Z * math.cos(gamma))
            
            if current_beat < 6 and leg_name == tap_sequence[current_beat]:
                local_y = base_y - (tap_height * arc_multiplier)
                local_x += (tap_extend * arc_multiplier)
                
            ik(local_x, local_y, local_z, leg_name, frame_angles)

        # DOWN-SAMPLING GATE: 240Hz PyBullet loop / 4 = 60Hz Hardware updates
        if t % 4 == 0:
            send_hardware(frame_angles)

        p.stepSimulation()
        time.sleep(1./240.)

# --- 5. INITIALIZATION SEQUENCE ---
for leg in LEGS.values():
    p.resetJointState(bot, leg[1], targetValue=130*math.pi/180)
    p.resetJointState(bot, leg[2], targetValue=100*math.pi/180)

print("Moving to default safe stance. Verify hardware positions now...")
frame_angles = {}
for leg_name in LEGS.keys():
    ik(x1_low, y1_low, 0, leg_name, frame_angles)
    
# Keep sending the safe baseline stance for 3 seconds before starting the dance
for safe_frame in range(240 * 3): 
    if safe_frame % 4 == 0:
        send_hardware(frame_angles)
    p.stepSimulation()
    time.sleep(1./240.)

# Run Main Routine
dance_macarena()

print("Routine complete. Entering loop to keep simulation alive.")
while True:
    p.stepSimulation()
    time.sleep(1./240.)