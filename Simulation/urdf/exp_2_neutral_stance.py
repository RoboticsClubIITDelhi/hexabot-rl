import math

import pybullet as p
import time
import pybullet_data    

LEG_JOINT_NAMES = {
    "L1": ("L1_1",        "Revolute 102", "Revolute 100"),
    "L2": ("Revolute 63", "Revolute 73",  "Revolute 90"),
    "L3": ("Revolute 62", "Revolute 72",  "Revolute 97"),
    "L4": ("Revolute 61", "Revolute 71",  "Revolute 95"),
    "L5": ("Revolute 60", "Revolute 70",  "Revolute 80"),
    "L6": ("Revolute 59", "Revolute 69",  "Revolute 92"),
}
#Constants
L1,L2,L3=0.074, 0.049, 0.0924
a=(24.6 * math.pi )/ 180.0
b=(76.3 * math.pi )/ 180.0

body_height=0.08
neutral_reach=0.13

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
        coxa, femur, tibia = LEG_JOINT_NAMES[leg]
        t1, t2, t3 = calculateIK(neutral_reach, 0, -body_height)
        p.setJointMotorControl2(robotId, mapperdict[coxa], p.POSITION_CONTROL, targetPosition=t1)
        p.setJointMotorControl2(robotId, mapperdict[femur], p.POSITION_CONTROL, targetPosition=t2)
        p.setJointMotorControl2(robotId, mapperdict[tibia], p.POSITION_CONTROL, targetPosition=t3)

        
        
while True:
    neutralstance() 
    # print(p.getJointInfo(robotId, 0))  
 
    p.stepSimulation() 
    time.sleep(1./240.)  