import numpy as np
from math import *
import time
from pyquaternion import Quaternion
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)

pca.reset()
pca.frequency = 100
servo0 = servo.Servo(pca.channels[0],min_pulse=700,max_pulse=2300,actuation_range=180)
servo1 = servo.Servo(pca.channels[1],min_pulse=700,max_pulse=2300,actuation_range=180)
servo2 = servo.Servo(pca.channels[2],min_pulse=700,max_pulse=2300,actuation_range=180)
servo3 = servo.Servo(pca.channels[3],min_pulse=700,max_pulse=2300,actuation_range=180)
servo4 = servo.Servo(pca.channels[4],min_pulse=700,max_pulse=2300,actuation_range=180)
servo5 = servo.Servo(pca.channels[5],min_pulse=700,max_pulse=2300,actuation_range=180)
servo6 = servo.Servo(pca.channels[6],min_pulse=700,max_pulse=2300,actuation_range=180)
servo7 = servo.Servo(pca.channels[7],min_pulse=700,max_pulse=2300,actuation_range=180)
servo8 = servo.Servo(pca.channels[8],min_pulse=700,max_pulse=2300,actuation_range=180)
servo9 = servo.Servo(pca.channels[9],min_pulse=700,max_pulse=2300,actuation_range=180)

motor_direction = np.array([1, 1, -1, 1, -1,     1, -1, 1, -1, -1])
initial_angle = 90.0
angle_offset = np.array([-2, 2.14, -2.0, 10, 0.,    -6, 2, 0, -8, 0.])

def publish_all():
    # rhip_y, rhip_x, rknee, rankle_x, rankle_y, lhip_y, lhip_x, lknee, lankle_x, ankle_y
    raw_cmd = np.array([rth,rht,rts,rsa,raf,lth,lht,lts,lsa,laf])
    cmd = raw_cmd*motor_direction+initial_angle+angle_offset
    servo0.angle = cmd[0]
    servo1.angle = cmd[1]
    servo2.angle = cmd[2]
    servo3.angle = cmd[3]
    servo4.angle = cmd[4]
    servo5.angle = cmd[5]
    servo6.angle = cmd[6]
    servo7.angle = cmd[7]
    servo8.angle = cmd[8]
    servo9.angle = cmd[9]
    
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X =  atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y =  asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z =  atan2(t3, t4)
    return X, Y, Z
lrHipHeightDiff = 0
Q = Quaternion(1, 0, 0, 0)
pitchx,rolly = [0,0]

def Reset():
    global lht,lth,lts,lsa,laf,rht,rth,rts,rsa,raf,lpos,rpos,lvel,lacc,rvel,racc
    lpos = np.array([0.0,0.0,270.0])
    rpos = np.array([0.0,0.0,270.0])
    lvel = np.zeros(3)
    rvel = np.zeros(3)
    lacc = np.zeros(3)
    racc = np.zeros(3)
    lth,lht,lts,lsa,laf= torso_based_ik(lpos,True)
    rth,rht,rts,rsa,raf= torso_based_ik(rpos,True)
    publish_all()
    time.sleep(3)

def torso_based_ik(pos,reseting=False):
    x,y,raw_z=pos
    sqsum = x**2+y**2+raw_z**2
    if sqsum>280**2:
        x/=sqrt(sqsum/280**2)
        y/=sqrt(sqsum/280**2)
        raw_z/=sqrt(sqsum/280**2)
    thigh=80
    shank=85
    ankle_height=45
    foot_height=17
    new_ankle_height=0
    old_ankle_height=45
    idx=0
    for _ in range(3):
        if abs(new_ankle_height-old_ankle_height)<1:
            break
        idx+=1
        new_ankle_height = old_ankle_height
        z = raw_z-new_ankle_height - foot_height
        l=sqrt(x**2+y**2+z**2)
        alpha = asin(y/l)
        l1 = sqrt(   max(   (l*cos(alpha)-55)**2+y**2,  0   )       )
        knee = acos(    np.clip ( (thigh**2+shank**2-l1**2)/(2*thigh*shank), -1,1)        )-np.pi
        hip_x = -asin(   np.clip (y/l1,-1,1))  + acos(    np.clip((l1**2+thigh**2-shank**2)/(2*l1*thigh),-1,1)           ) ######### add - 
        old_ankle_height = ankle_height*cos(hip_x)
    hip_y = asin(   np.clip   (x/l/cos(alpha),-1,1)      )
    hip_x*=180/np.pi 
    hip_y*=-180/np.pi 
    knee*=180/np.pi
    if reseting:
        ankle_x = -(hip_x+knee)
        ankle_y = -hip_y 
    else:
        ankle_x = -(hip_x+knee)-pitchx*0.7
        ankle_y = -hip_y - rolly*0.8
    return hip_y,hip_x,knee,ankle_x,ankle_y


lht,lth,lts,lsa,laf,rht,rth,rts,rsa,raf = np.zeros(10) 
lpos = np.array([0.0,0.0,270.0])
rpos = np.array([0.0,0.0,270.0])
lvel = np.zeros(3)
rvel = np.zeros(3)
lacc = np.zeros(3)
racc = np.zeros(3)
rospy.sleep(1.5)
lth,lht,lts,lsa,laf= torso_based_ik(lpos)
rth,rht,rts,rsa,raf= torso_based_ik(rpos)
g=9.81
dt=0.02
publish_all()
Reset()




# stand to walk transition, double support px = x-ax*z/(g+az)
T_start = time.time()
Timer=0
while not rospy.is_shutdown() and time.time()-T_start<0.5:
    if time.time()-Timer>dt:
        Timer = time.time()
        if time.time()-T_start<0.25:
            lacc = np.array([-300.0, 0.0, -50.0])  # -20mm*16
            racc = np.array([-300.0, 0.0, -50.0])
        else:
            lacc = np.array([300.0, 0.0, 50.0]) 
            racc = np.array([300.0, 0.0, 50.0])
        lvel+=lacc*dt
        rvel+=racc*dt
        lpos+=lvel*dt+0.5*lacc*dt**2
        rpos+=rvel*dt+0.5*racc*dt**2
        lth,lht,lts,lsa,laf= torso_based_ik(lpos)
        rth,rht,rts,rsa,raf= torso_based_ik(rpos)
    publish_all()
    rospy.sleep(0.001)



step_height = 225
swing_height=270

SwingForwardAcc = 800 
StanceForwardAccBase = -600
StanceForwardAccmlt = 100

LaterialStanceAcc=600
# main walking loop 
while not rospy.is_shutdown():
    # LIFT right foot & keep or gain forward speed ############################
    T_start = time.time()
    Timer=0
    lvel = rvel
    rvel = np.zeros(3)
    rvel[2] = -10
    while not rospy.is_shutdown() and time.time()-T_start<0.2:
        if time.time()-Timer>dt:
            lacc = -50*(lpos-np.array([-32, 5, swing_height]))-5*lvel

            racc = -100*(rpos-np.array([-32, 0, step_height]))-10*rvel
            Timer = time.time()
            lvel+=lacc*dt
            rvel+=racc*dt
            lpos+=lvel*dt+0.5*lacc*dt**2
            rpos+=rvel*dt+0.5*racc*dt**2
            lth,lht,lts,lsa,laf= torso_based_ik(lpos)
            rth,rht,rts,rsa,raf= torso_based_ik(rpos)
        publish_all()
        rospy.sleep(0.001)



    # left stance right swing 
    T_start = time.time()
    Timer=0
    rvel = np.zeros(3)
    lvel = np.zeros(3)
    lacc = np.array([LaterialStanceAcc, SwingForwardAcc, 0.0]) 
    racc = np.array([0.0, StanceForwardAccBase,  0.0])
    rvel[2] = 100
    while not rospy.is_shutdown() and Q.rotate([0,0,rpos[2]])[2]< Q.rotate([0,0,lpos[2]])[2]+lrHipHeightDiff:
        if time.time()-Timer>dt:
            Timer = time.time()

            racc[0] = - 2*rvel[0]-20*(rpos[0]-1) 

            racc[1] = StanceForwardAccBase - StanceForwardAccmlt*rpos[1] 

            lvel+=lacc*dt
            rvel+=racc*dt
            lpos+=lvel*dt+0.5*lacc*dt**2
            rpos+=rvel*dt+0.5*racc*dt**2
            lth,lht,lts,lsa,laf= torso_based_ik(lpos)
            rth,rht,rts,rsa,raf= torso_based_ik(rpos)
        publish_all()
        rospy.sleep(0.001)

    while not rospy.is_shutdown() and time.time()-T_start<0.01:
        rospy.sleep(0.001)


    # lift left foot & keep or gain forward speed ##################################
    T_start = time.time()
    Timer=0
    rvel = lvel
    lvel = np.zeros(3)
    lvel[2] = -10
    while not rospy.is_shutdown() and time.time()-T_start<0.2:
        if time.time()-Timer>dt:

            
            lacc = -100*(lpos-np.array([32,0,step_height]))-10*lvel

            racc = -50*(rpos-np.array([32,5,swing_height]))-5*rvel

            Timer = time.time()
            lvel+=lacc*dt
            rvel+=racc*dt
            lpos+=lvel*dt+0.5*lacc*dt**2
            rpos+=rvel*dt+0.5*racc*dt**2
            lth,lht,lts,lsa,laf= torso_based_ik(lpos)
            rth,rht,rts,rsa,raf= torso_based_ik(rpos)
        publish_all()
        rospy.sleep(0.001)


 

    # right stance left swing 
    T_start = time.time()
    Timer=0
    rvel = np.zeros(3)
    lvel = np.zeros(3)
    lacc = np.array([0.0, StanceForwardAccBase,  0.0])
    racc = np.array([-LaterialStanceAcc, SwingForwardAcc, 0.0]) 
    lvel[2] = 100
    while not rospy.is_shutdown() and Q.rotate([0,0,lpos[2]])[2]<Q.rotate([0,0,rpos[2]])[2]-lrHipHeightDiff:
        if time.time()-Timer>dt:
            Timer = time.time()

            lacc[0] = - 2*lvel[0]-20*(lpos[0]+1)

            lacc[1] = StanceForwardAccBase - StanceForwardAccmlt*lpos[1]

            lvel+=lacc*dt
            rvel+=racc*dt
            lpos+=lvel*dt+0.5*lacc*dt**2
            rpos+=rvel*dt+0.5*racc*dt**2
            lth,lht,lts,lsa,laf= torso_based_ik(lpos)
            rth,rht,rts,rsa,raf= torso_based_ik(rpos)
        publish_all()
        rospy.sleep(0.001)
    
    T_start = time.time()
    while not rospy.is_shutdown() and time.time()-T_start<0.01:
        rospy.sleep(0.001)

    



