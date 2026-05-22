import pybullet as p
import time
import pybullet_data
import math
from serial import Serial

try:
    ser = Serial('COM7', 230400, timeout=0.01)
    print("Connected to ESP32 at 230400 baud")
except Exception as e:
    print(f"Serial error: {e}")
    exit()

LEGS = {
    'L1': [1, 3, 5],
    'L2': [8, 10, 12],
    'L3': [15, 17, 19],
    'L4': [22, 24, 26],
    'L5': [30, 32, 34],
    'L6': [37, 39, 41]
}

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=10.0)
mu = p.getQuaternionFromEuler([0, 0, 0])
bot = p.loadURDF("robot.urdf", [0, 0, 0.30], useFixedBase=False, baseOrientation=mu)
mode = p.POSITION_CONTROL
p.setGravity(0, 0, -10)

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

def angle_to_step(rad, sign=1):
    deg = math.degrees(rad * sign)
    step = int(round(512 + deg * STEPS_PER_DEG))
    return max(0, min(1023, step))

def send_hardware(leg_angles):
    parts = []
    for leg, (l, m, n) in leg_angles.items():
        ids   = HW_IDS[leg]
        signs = HW_SIGNS[leg]
        steps = [
            angle_to_step(l,  signs[0]),
            angle_to_step(m,  signs[1]),
            angle_to_step(-n, signs[2]),   
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
for i in LEGS:
    p.resetJointState(bot, LEGS[i][1], targetValue=130*math.pi/180)
    p.resetJointState(bot, LEGS[i][2], targetValue=100*math.pi/180)
a = 74.015
b = 49.064
c = 96.933
d = 218
maxdis = 60
maxlev = 0
x  = 24.6 * math.pi / 180
y  = 68.2 * math.pi / 180
x1 = a + b * math.sin(x+130*math.pi/180) + c * math.cos(y+30*math.pi/180)
y1 = b * math.cos(x+130*math.pi/180) - c * math.sin(y+30*math.pi/180)
R1_2  = (x1 - a)**2 + y1**2
phi_1 = math.atan2(y1, x1 - a)
theta_1 = x - y
beta_1  = math.atan2(b + c * math.sin(theta_1), c * math.cos(theta_1))
x41= x1-maxdis/2
z41= 0-math.sqrt(3)*maxdis/2
y41= y1- (d/4)*math.sin(maxlev)
x2_y2_z2= {
    'L1': [(x1-maxdis)*math.cos(maxlev)-(y1-(d/2)*math.sin(maxlev))*math.sin(maxlev), (x1-maxdis)*math.sin(maxlev)+(y1-(d/2)*math.sin(maxlev))*math.cos(maxlev), 0],
    'L2': [x41*(math.cos(maxlev)*0.25+0.75)-y41*0.5*math.sin(maxlev)-z41*(math.sqrt(3)/4)*(1-math.cos(maxlev)), x41*(math.sin(maxlev))*1/2+y41*math.cos(maxlev)+(z41*math.sqrt(3)/2)*math.sin(maxlev), -x41*(math.sqrt(3)/4)*(1-math.cos(maxlev))-(y41*math.sqrt(3)/2)*math.sin(maxlev)+z41*(math.cos(maxlev)*0.75+0.25)],
    'L3': [x41*(math.cos(maxlev)*0.25+0.75)+y41*1/2*math.sin(maxlev)+z41*(math.sqrt(3)/4)*(1-math.cos(maxlev)), -x41*(math.sin(maxlev))*1/2+y41*math.cos(maxlev)+(z41*math.sqrt(3)/2)*math.sin(maxlev), x41*(math.sqrt(3)/4)*(1-math.cos(maxlev))-(y41*math.sqrt(3)/2)*math.sin(maxlev)+z41*(math.cos(maxlev)*0.75+0.25)],
    'L4': [(x1+maxdis)*math.cos(maxlev)+(y1+(d/2)*math.sin(maxlev))*math.sin(maxlev), -(x1+maxdis)*math.sin(maxlev)+(y1+(d/2)*math.sin(maxlev))*math.cos(maxlev), 0],
    'L5': [x41*(math.cos(maxlev)*0.25+0.75)+y41*1/2*math.sin(maxlev)+z41*(math.sqrt(3)/4)*(1-math.cos(maxlev)), -x41*(math.sin(maxlev))*1/2+y41*math.cos(maxlev)+(z41*math.sqrt(3)/2)*math.sin(maxlev), -(x41*(math.sqrt(3)/4)*(1-math.cos(maxlev))-(y41*math.sqrt(3)/2)*math.sin(maxlev)+z41*(math.cos(maxlev)*0.75+0.25))],
    'L6': [x41*(math.cos(maxlev)*0.25+0.75)-y41*0.5*math.sin(maxlev)-z41*(math.sqrt(3)/4)*(1-math.cos(maxlev)), x41*(math.sin(maxlev))*1/2+y41*math.cos(maxlev)+(z41*math.sqrt(3)/2)*math.sin(maxlev), -(-x41*(math.sqrt(3)/4)*(1-math.cos(maxlev))-(y41*math.sqrt(3)/2)*math.sin(maxlev)+z41*(math.cos(maxlev)*0.75+0.25))]
}

def ik(x2,y2,z2,leg,frame_angles):
    l= math.atan2(-z2,x2)
    R2_2= (math.sqrt(x2**2+z2**2)-a)**2+y2**2
    r=(b**2+c**2-R2_2)/(2*b*c)
    if r>1:
        n= math.pi/2+x-y
    elif r<-1:
        n= math.pi/2+x-y-math.pi
    else:
        n= math.pi/2+x-y- math.acos(r)
    q= (-c**2+b**2+R2_2)/(2*b*math.sqrt(R2_2))
    if q>1:
        sup=0
    elif q<-1:
        sup=math.pi
    else:
        sup= math.acos(q)
    pu= (a**2+R2_2-x2**2-y2**2-z2**2)/(2*a*math.sqrt(R2_2))
    if pu>1:
        m= 3*math.pi/2-x-sup
    elif pu<-1:
        m= 3*math.pi/2-x-sup-math.pi
    else:
        m= 3*math.pi/2-x-sup-math.acos(pu)
    p.setJointMotorControl2(bot, LEGS[leg][0], controlMode=mode,targetPosition=l, force=1000, maxVelocity=1000)
    p.setJointMotorControl2(bot, LEGS[leg][1], controlMode=mode,targetPosition=m, force=1000, maxVelocity=1000)
    p.setJointMotorControl2(bot, LEGS[leg][2], controlMode=mode,targetPosition=-n, force=1000, maxVelocity=1000)
    frame_angles[leg] = (-l, -m, n)
spd=240
def bz(t,leg,frame_angles):
    s=t/spd
    x2= x2_y2_z2[leg][0]
    y2= x2_y2_z2[leg][1]
    z2= x2_y2_z2[leg][2]
    ik(x1+(x2-x1)*s, y1*(1-s)**2 + y2*s**2, z2*s, leg, frame_angles)
i = 0
k= 0
while True:
    if i<500:
        p.stepSimulation()
        time.sleep(1./240.)
        i+=1
        continue
    if i>500+2*spd:
        i=501
    theta = maxlev*((math.pi)/180)*math.sin(math.pi*(i-500)/spd)
    dis= maxdis* math.sin(math.pi*(i-500)/spd)   
    phase= (i-500)%(2*spd)
    if phase<spd:
        dis= maxdis* math.sin(math.pi*(i+spd-500)/spd)
        x2= (x1-dis)*math.cos(theta)-(y1-(d/2)*math.sin(theta))*math.sin(theta)
        y2= (x1-dis)*math.sin(theta)+(y1-(d/2)*math.sin(theta))*math.cos(theta)
        x42= x1+dis/2
        z42= -math.sqrt(3)*dis/2
        y42= y1+ (d/4)*math.sin(theta)
        x5= x42*(math.cos(theta)*0.25+0.75)+y42*1/2*math.sin(theta)+z42*(math.sqrt(3)/4)*(1-math.cos(theta))
        y5= -x42*(math.sin(theta))*1/2+y42*math.cos(theta)+(z42*math.sqrt(3)/2)*math.sin(theta)
        z5= x42*(math.sqrt(3)/4)*(1-math.cos(theta))-(y42*math.sqrt(3)/2)*math.sin(theta)+z42*(math.cos(theta)*0.75+0.25)
        frame_angles = {}
        ik(x2,y2+0.1,0,'L1',frame_angles)
        ik(x5,y5,-z5,'L5',frame_angles)
        ik(x5,y5,z5,'L3',frame_angles)
        bz(phase+1, 'L2',frame_angles)
        bz(phase+1, 'L4',frame_angles)
        bz(phase+1, 'L6',frame_angles)
        send_hardware(frame_angles)
    if phase>=spd:
        x3= (x1+dis)*math.cos(theta)+(y1+(d/2)*math.sin(theta))*math.sin(theta)
        y3= -(x1+dis)*math.sin(theta)+(y1+(d/2)*math.sin(theta))*math.cos(theta)
        x42= x1-dis/2
        z42= -math.sqrt(3)*dis/2
        y42= y1- (d/4)*math.sin(theta)
        x4= x42*(math.cos(theta)*0.25+0.75)-y42*0.5*math.sin(theta)-z42*(math.sqrt(3)/4)*(1-math.cos(theta))
        y4= x42*(math.sin(theta))*1/2+y42*math.cos(theta)+(z42*math.sqrt(3)/2)*math.sin(theta)
        z4= -x42*(math.sqrt(3)/4)*(1-math.cos(theta))-(y42*math.sqrt(3)/2)*math.sin(theta)+z42*(math.cos(theta)*0.75+0.25)
        frame_angles = {}
        ik(x3,y3+0.1,0,'L4', frame_angles)
        ik(x4,y4,z4,'L2',frame_angles)
        ik(x4,y4,-z4,'L6',frame_angles)
        bz(phase-spd+1, 'L1', frame_angles)
        bz(phase-spd+1, 'L3',frame_angles)
        bz(phase-spd+1, 'L5',frame_angles)
        send_hardware(frame_angles)
    p.stepSimulation()
    time.sleep(1./240.)
    if k==0:
        time.sleep(3)
        k+=1
    i+=1
