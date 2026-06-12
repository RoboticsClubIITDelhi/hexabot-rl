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
a=24.6 * math.pi / 180.0
b=76.3 * math.pi / 180.0

body_height=0.09
neutral_reach=0.150 

def calculateIK(x, y, z):   
    theta1=math.atan2(y, x)
    r=math.sqrt(x**2 + y**2) - L1
    theta3=math.pi-math.acos((r**2 + z**2 - L2**2 - L3**2) / (2 * L2 * L3))
    theta2=math.atan2((-L3 * math.sin(theta3)*r-z*(L2+L3 * math.cos(theta3))), (L2 + L3 * math.cos(theta3)*r-z*(L3 * math.sin(theta3))))
    return theta1, theta2-a, theta3+b



physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf
p.setGravity(0, 0, -9.81)           
planeId = p.loadURDF("plane.urdf")  # load a plane

robotId = p.loadURDF("robot.urdf",[0,0,0.1])  # load a robot

print(p.getNumJoints(robotId))  # print the number of joints in the robot
L=[]
for i in range(p.getNumJoints(robotId)):
    if p.getJointInfo(robotId, i)[2] == p.JOINT_REVOLUTE:
        print(p.getJointInfo(robotId, i)[1])  # print joint information
        L.append(p.addUserDebugParameter(p.getJointInfo(robotId, i)[1].decode('utf-8'), -3.14, 3.14, 0.0))  # add a slider for this joint

        
        
while True:
    p.setJointMotorControlArray(robotId, [j for j in range(p.getNumJoints(robotId)) if p.getJointInfo(robotId, j)[2] == p.JOINT_REVOLUTE], p.POSITION_CONTROL, targetPositions=[p.readUserDebugParameter(L[x]) for x in range(len(L))])  # control the revolute joints to a position
    print([p.readUserDebugParameter(L[x]) for x in range(len(L))])  # read the values from the sliders
    p.stepSimulation()  # step the simulation
    time.sleep(1./240.)  # sleep to match real-time simulation