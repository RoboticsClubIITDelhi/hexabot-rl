import math

import pybullet as p
import time
import pybullet_data    

LEG_JOINT_NAMES = {
    "L1": ("L1_1",        "Revolute 102", "Revolute 100",0.0),
    "L2": ("Revolute 63", "Revolute 73",  "Revolute 90",60),
    "L3": ("Revolute 62", "Revolute 72",  "Revolute 97",120),
    "L4": ("Revolute 61", "Revolute 71",  "Revolute 95",180),
    "L5": ("Revolute 60", "Revolute 70",  "Revolute 80",-120),
    "L6": ("Revolute 59", "Revolute 69",  "Revolute 92",-60),
}
#Constants
L1,L2,L3=0.074, 0.049, 0.0924
a=(24.6 * math.pi )/ 180.0
b=(76.3 * math.pi )/ 180.0

body_height=0.08
neutral_reach=0.13

step_height=0.06
step_length=0.03

def calculateIK(x, y, z):   
    theta1=math.atan2(y, x)
    r=math.sqrt(x**2 + y**2) - L1
    costheta3=((r**2 + z**2 - L2**2 - L3**2) / (2 * L2 * L3))
    sintheta3=math.sqrt(1 - costheta3**2)
    theta3=math.atan2(sintheta3, costheta3)
    theta2 = math.atan2(
    -L3 * math.sin(theta3) * r - (L2 + L3 * math.cos(theta3)) * z, 
    (L2 + L3 * math.cos(theta3)) * r - L3 * math.sin(theta3) * z  )
   
    print(theta1, (math.pi/2)+theta2-a, -(theta3-(math.pi/2)+a-b))
    # print(theta1, theta2, theta3,a,b)
    return theta1, ((math.pi/2)+theta2-a), -(theta3-(math.pi/2)+a-b)

def beizer(leg,cycles):
    cos=math.cos(LEG_JOINT_NAMES[leg][3]*math.pi/180)
    sin=math.sin(LEG_JOINT_NAMES[leg][3]*math.pi/180)
    s=step_length/2
    p0=(neutral_reach-s*cos,-s*sin, -body_height)
    p1=(neutral_reach-s*cos/3,-s*sin/3, -body_height+step_height/2 )
    p2=(neutral_reach+s*cos/3,s*sin/3, -body_height+step_height/2)
    p3=(neutral_reach+s*cos,s*sin, -body_height)
    
    # p0=(neutral_reach,0, -body_height)
    # p1=(neutral_reach+(step_length*math.cos(LEG_JOINT_NAMES[leg][3]*math.pi/180))/3,step_length*(1/3)*math.sin(LEG_JOINT_NAMES[leg][3]*math.pi/180), -body_height+step_height/2)
    # p2=(neutral_reach+(2*step_length*math.cos(LEG_JOINT_NAMES[leg][3]*math.pi/180))/3,step_length*(2/3)*math.sin(LEG_JOINT_NAMES[leg][3]*math.pi/180), -body_height+step_height/2)
    # p3=(neutral_reach+step_length*math.cos(LEG_JOINT_NAMES[leg][3]*math.pi/180),step_length*math.sin(LEG_JOINT_NAMES[leg][3]*math.pi/180), -body_height)
    beizer_points=[]
    for i in range(cycles):
        t= i/cycles
        x= (1-t)**3*p0[0]+ 3*(1-t)**2*t*p1[0]+ 3*(1-t)*t**2*p2[0]+ t**3*p3[0]
        y= (1-t)**3*p0[1]+ 3*(1-t)**2*t*p1[1]+ 3*(1-t)*t**2*p2[1]+ t**3*p3[1]
        z= (1-t)**3*p0[2]+ 3*(1-t)**2*t*p1[2]+ 3*(1-t)*t**2*p2[2]+ t**3*p3[2]
        print(x,y,z)
        beizer_points.append((x,y,z))
    return beizer_points

def lin(leg,cycles):
    lin_points=[]
    for i in range(cycles):
        t= i/cycles
        x= neutral_reach+ step_length*(1-t)*math.cos(LEG_JOINT_NAMES[leg][3]*math.pi/180)
        y= step_length*(1-t)*math.sin(LEG_JOINT_NAMES[leg][3]*math.pi/180)
        z= -body_height
        print(x,y,z)
        lin_points.append((x,y,z))
    return lin_points
        


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

def neutralstance():
    for leg in LEG_JOINT_NAMES:
        coxa, femur, tibia,angle= LEG_JOINT_NAMES[leg]
        t1, t2, t3 = calculateIK(neutral_reach, 0, -body_height)
        p.setJointMotorControl2(robotId, mapperdict[coxa], p.POSITION_CONTROL, targetPosition=t1)
        p.setJointMotorControl2(robotId, mapperdict[femur], p.POSITION_CONTROL, targetPosition=t2)
        p.setJointMotorControl2(robotId, mapperdict[tibia], p.POSITION_CONTROL, targetPosition=t3)
cycles=50
tri = [
    ("L1", beizer("L1", cycles)),
    ("L3", beizer("L3", cycles)),
    ("L5", beizer("L5", cycles))
]    
i=0
odd=True
while True:
    neutralstance() 
    if i>=cycles:
        i=0
        odd= not odd
    if odd:
        tri = [
            ("L1", beizer("L1", cycles)),
            ("L3", beizer("L3", cycles)),
            ("L5", beizer("L5", cycles))
        ]

    else:
         tri = [
            ("L1", lin("L1", cycles)),
            ("L3", lin("L3", cycles)),
            ("L5", lin("L5", cycles))
        ]
       
   
    for leg in tri:
        coxa, femur, tibia,angle= LEG_JOINT_NAMES[leg[0]]
        t1, t2, t3 = calculateIK(leg[1][i][0], leg[1][i][1], leg[1][i][2])
        p.setJointMotorControl2(robotId, mapperdict[coxa], p.POSITION_CONTROL, targetPosition=t1)
        p.setJointMotorControl2(robotId, mapperdict[femur], p.POSITION_CONTROL, targetPosition=t2)
        p.setJointMotorControl2(robotId, mapperdict[tibia], p.POSITION_CONTROL, targetPosition=t3)
            
           
            
        
    i+=1
    # print(p.getJointInfo(robotId, 0))  
 
    p.stepSimulation() 
    time.sleep(1./240.)  