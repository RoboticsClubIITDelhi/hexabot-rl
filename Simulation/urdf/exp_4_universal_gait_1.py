import math

import pybullet as p
import time
import pybullet_data    

LEG_JOINT_NAMES = {
    "L1": ("L1_1",        "Revolute 102", "Revolute 100",0),
    "L2": ("Revolute 63", "Revolute 73",  "Revolute 90",-60),
    "L3": ("Revolute 62", "Revolute 72",  "Revolute 97",-120),
    "L4": ("Revolute 61", "Revolute 71",  "Revolute 95",180),
    "L5": ("Revolute 60", "Revolute 70",  "Revolute 80",120),
    "L6": ("Revolute 59", "Revolute 69",  "Revolute 92",60),
}


#Constants
L1,L2,L3=0.074, 0.049, 0.0924
a=(24.6 * math.pi )/ 180.0
b=(76.3 * math.pi )/ 180.0

#Parameters for gait and neutral stance

body_height=0.08
neutral_reach=0.14

step_height=0.08
step_length=0.06



def calculateIK(x, y, z):   
    theta1=math.atan2(y, x)
    r=math.sqrt(x**2 + y**2) - L1
    costheta3=((r**2 + z**2 - L2**2 - L3**2) / (2 * L2 * L3))
    sintheta3=math.sqrt(1 - costheta3**2)
    theta3=math.atan2(sintheta3, costheta3)
    theta2 = math.atan2(
    -L3 * math.sin(theta3) * r - (L2 + L3 * math.cos(theta3)) * z, 
    (L2 + L3 * math.cos(theta3)) * r - L3 * math.sin(theta3) * z  )
   
    # print(theta1, (math.pi/2)+theta2-a, -(theta3-(math.pi/2)+a-b))
    # print(theta1, theta2, theta3,a,b)
    return theta1, ((math.pi/2)+theta2-a), -(theta3-(math.pi/2)+a-b)

def beizer(leg,cycles,t):
    cos=math.cos(LEG_JOINT_NAMES[leg][3]*math.pi/180)
    sin=math.sin(LEG_JOINT_NAMES[leg][3]*math.pi/180)
    s=step_length/2
    
    p0=(neutral_reach-s*cos,-s*sin, -body_height)
    p1=(neutral_reach-s*cos/3,-s*sin/3, -body_height+step_height/2 )
    p2=(neutral_reach+s*cos/3,s*sin/3, -body_height+step_height/2)
    p3=(neutral_reach+s*cos,s*sin, -body_height)
    t= t/float(cycles - 1)
    x= (1-t)**3*p0[0]+ 3*(1-t)**2*t*p1[0]+ 3*(1-t)*t**2*p2[0]+ t**3*p3[0]
    y= (1-t)**3*p0[1]+ 3*(1-t)**2*t*p1[1]+ 3*(1-t)*t**2*p2[1]+ t**3*p3[1]
    z= (1-t)**3*p0[2]+ 3*(1-t)**2*t*p1[2]+ 3*(1-t)*t**2*p2[2]+ t**3*p3[2]
    print(x,y,z)
    return x, y, z

def lin(leg,cycles,t):
    cos=math.cos(LEG_JOINT_NAMES[leg][3]*math.pi/180)
    sin=math.sin(LEG_JOINT_NAMES[leg][3]*math.pi/180)
    t= t/float(cycles - 1)
    x= neutral_reach-(step_length/2)*cos+ step_length*(1-t)*cos
    y= step_length*(1-t)*sin -(step_length/2)*sin
    z= -body_height
    print(x,y,z)
    return x, y, z

def neutralstance():
    for leg in LEG_JOINT_NAMES:
        coxa, femur, tibia,angle= LEG_JOINT_NAMES[leg]
        t1, t2, t3 = calculateIK(neutral_reach, 0, -body_height)
        p.setJointMotorControl2(robotId, mapperdict[coxa], p.POSITION_CONTROL, targetPosition=t1)
        p.setJointMotorControl2(robotId, mapperdict[femur], p.POSITION_CONTROL, targetPosition=t2)
        p.setJointMotorControl2(robotId, mapperdict[tibia], p.POSITION_CONTROL, targetPosition=t3)



#setup pybullet code
physicsClient = p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())  
p.setGravity(0, 0, -9.81)           
planeId = p.loadURDF("plane.urdf") 

robotId = p.loadURDF("robot.urdf",[0,0,0.1]) 
mapperdict={}
print(p.getNumJoints(robotId)) 
for i in range(p.getNumJoints(robotId)):
    if p.getJointInfo(robotId, i)[2] == p.JOINT_REVOLUTE:
        p.changeDynamics(robotId, i, jointLowerLimit=-math.pi, jointUpperLimit=math.pi) 
        mapperdict[p.getJointInfo(robotId, i)[1].decode('utf-8')]=p.getJointInfo(robotId, i)[0]  


#gait parameters 

cycles=200
i=0

GAIT_LIBRARY = {
    "pronk": {
        "phase_shifts": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "duty_factor": 0.50
    },
    "wave": {
        "phase_shifts": [0.0, 2/6, 4/6, 1/6, 3/6, 5/6],
        "duty_factor": 5/6 
    },
    "metachronal_tripod": {
        "phase_shifts": [0.0, 5/6, 4/6, 3/6, 2/6, 1/6],
        "duty_factor": 4/6 
    },
    "rolling_tripod": {
        "phase_shifts": [0.0, 2/6, 4/6, 3/6, 5/6, 1/6],
        "duty_factor": 4/6  
    },
    "tetrapod": {
        "phase_shifts": [0.0, 3/4, 1/2, 1/2, 1/4, 0.0],
        "duty_factor": 3/4 
    },
    "bowtie": {
        "phase_shifts": [0.0, 1/2, 1/2, 1/4, 3/4, 3/4],
        "duty_factor": 3/4 
    },
    "caterpillar": {
        "phase_shifts": [0.0, 2/3, 1/3, 0.0, 2/3, 1/3],
        "duty_factor": 2/3  
    },
    "rice": {
        "phase_shifts": [0.0, 2/3, 1/3, 1/3, 2/3, 0.0],
        "duty_factor": 2/3  
    },
    "tripod": {
        "phase_shifts": [0.0, 0.5, 0.0, 0.5, 0.0, 0.5],
        "duty_factor": 0.50 
    },
    "pace": {
        "phase_shifts": [0.0, 0.0, 0.0, 0.5, 0.5, 0.5],
        "duty_factor": 0.50 
    }
}

neutralstance()

#can change the gait by changing the key here 

phase_shifts = GAIT_LIBRARY["pronk"]["phase_shifts"]
duty_factor = GAIT_LIBRARY["pronk"]["duty_factor"]

while True:
    
    if i>=cycles:
        i=0

    for k in range(len(LEG_JOINT_NAMES)):
        leg='L'+str(k+1)
        coxa, femur, tibia,angle= LEG_JOINT_NAMES[leg]
        phase_shift= phase_shifts[k]
        phase= (i/cycles + phase_shift)%1
        if phase<(1-duty_factor):
            x,y,z= beizer(leg, int(cycles*(1-duty_factor)),int(phase*cycles))
        else:
            x,y,z= lin(leg, int(cycles*(duty_factor)),int((phase-(1-duty_factor))*cycles))
        t1, t2, t3 = calculateIK(x, y, z)
        p.setJointMotorControl2(robotId, mapperdict[coxa], p.POSITION_CONTROL, targetPosition=t1,force=6.0)
        p.setJointMotorControl2(robotId, mapperdict[femur], p.POSITION_CONTROL, targetPosition=t2,force=6.0)
        p.setJointMotorControl2(robotId, mapperdict[tibia], p.POSITION_CONTROL, targetPosition=t3,force=6.0)
 
    i+=1 
 
    p.stepSimulation() 
    time.sleep(1./240.)  