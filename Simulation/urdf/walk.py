import pybullet as p
import time
import pybullet_data
import math
LEGS = {
    'L1': [1, 3, 5,0,0], 
    'L2': [8, 10, 12,0,0], 
    'L3': [15, 17, 19,0,0], 
    'L4': [22, 24, 26,0,0], 
    'L5': [30, 32, 34,0,0], 
    'L6': [37, 39, 41,0,0]
}
URDF_PATH = "robot.urdf"
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=100.0) 
mu= p.getQuaternionFromEuler([0,0,0])
bot = p.loadURDF(URDF_PATH, [0, 0, 0.30], useFixedBase=False, baseOrientation=mu)
for i in LEGS:
    for j in range(3):
        p.enableJointForceTorqueSensor(bot, LEGS[i][j], enableSensor=1)

max= 10
mode = p.POSITION_CONTROL

p.setGravity(0, 0, -10)
for i in LEGS:
    p.resetJointState(bot, LEGS[i][1], targetValue=50*math.pi/180)
    p.resetJointState(bot, LEGS[i][2], targetValue=50*math.pi/180)
maxdis=70
maxlev=0*math.pi/180
i=0
a= 74.015
b= 49.064
c= (96.933+96.933)/2
d= 218
x=(24.6)*(math.pi)/180
y=(68.2)*math.pi/180
x1= a+b*math.sin(x+50*math.pi/180)+c*math.cos(y-0*math.pi/180)
y1= b*math.cos(x+50*math.pi/180)-c*math.sin(y-0*math.pi/180)
z1=0
R1_2= (x1-a)**2+y1**2
phi_1= math.atan2(y1,x1-a)
theta_1= x-y
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


beta_1= math.atan2(b+c*math.sin(theta_1), c*math.cos(theta_1))
def ik(x2,y2,z2,leg):
    if LEGS[leg][3]==1 and LEGS[leg][4]<25:
        LEGS[leg][4]+=1
    elif LEGS[leg][3]==1 and LEGS[leg][4]==25:
        LEGS[leg][3]=0
        LEGS[leg][4]=0
    else:
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
        p.setJointMotorControl2(bot, LEGS[leg][0], controlMode=mode,targetPosition=l, force=max, maxVelocity=1000)
        p.setJointMotorControl2(bot, LEGS[leg][1], controlMode=mode,targetPosition=m, force=max, maxVelocity=1000)
        p.setJointMotorControl2(bot, LEGS[leg][2], controlMode=mode,targetPosition=-n, force=max, maxVelocity=1000)
spd=90
def bz(t,leg):
    s=t/spd
    x2= x2_y2_z2[leg][0]
    y2= x2_y2_z2[leg][1]
    z2= x2_y2_z2[leg][2]
    ik(x1+(x2-x1)*s, y1*(1-s) + y2*s**2, z2*s, leg)

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
        ik(x2,y2+0.1,0,'L1')
        ik(x5,y5,-z5,'L5')
        ik(x5,y5,z5,'L3')
        bz(phase+1, 'L2')
        bz(phase+1, 'L4')
        bz(phase+1, 'L6')
        force= {}
        for j in LEGS:
            force[j]= p.getJointState(bot, LEGS[j][2])[2]
        for j in LEGS:
            if force[j][0]**2+force[j][1]**2+force[j][2]**2>10000:
                LEGS[j][3]=1
    if phase>=spd:
        x3= (x1+dis)*math.cos(theta)+(y1+(d/2)*math.sin(theta))*math.sin(theta)
        y3= -(x1+dis)*math.sin(theta)+(y1+(d/2)*math.sin(theta))*math.cos(theta)
        x42= x1-dis/2
        z42= -math.sqrt(3)*dis/2
        y42= y1- (d/4)*math.sin(theta)
        x4= x42*(math.cos(theta)*0.25+0.75)-y42*0.5*math.sin(theta)-z42*(math.sqrt(3)/4)*(1-math.cos(theta))
        y4= x42*(math.sin(theta))*1/2+y42*math.cos(theta)+(z42*math.sqrt(3)/2)*math.sin(theta)
        z4= -x42*(math.sqrt(3)/4)*(1-math.cos(theta))-(y42*math.sqrt(3)/2)*math.sin(theta)+z42*(math.cos(theta)*0.75+0.25)
        ik(x3,y3+0.1,0,'L4')
        ik(x4,y4,z4,'L2')
        ik(x4,y4,-z4,'L6')
        bz(phase-spd+1, 'L1')
        bz(phase-spd+1, 'L3')
        bz(phase-spd+1, 'L5')
        force= {}
        for j in LEGS:
            force[j]= p.getJointState(bot, LEGS[j][2])[2]
        for j in LEGS:
            if force[j][0]**2+force[j][1]**2+force[j][2]**2>10000:
                LEGS[j][3]=1
    p.stepSimulation()
    time.sleep(1./240.)
    i+=1
