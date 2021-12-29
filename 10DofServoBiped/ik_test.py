import time
import numpy as np
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from math import *

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

def motor_control(raw_cmd):
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
    
def torso_based_ik(x,y,raw_z):
    thigh=80
    shank=85
    ankle_height=45
    foot_height=17
    new_ankle_height=0
    old_ankle_height=45
    idx=0
    while abs(new_ankle_height-old_ankle_height)>1:
        idx+=1
        new_ankle_height = old_ankle_height
        z = raw_z-new_ankle_height - foot_height
        l=sqrt(x**2+y**2+z**2)
        alpha = asin(y/l)
        l1 = sqrt((l*cos(alpha)-55)**2+y**2)
        knee = acos((thigh**2+shank**2-l1**2)/(2*thigh*shank))-np.pi
        hip_x = asin(y/l1)+ acos((l1**2+thigh**2-shank**2)/(2*l1*thigh))
        old_ankle_height = ankle_height*cos(hip_x)
    hip_y = asin(x/l/cos(alpha))
    hip_x*=180/np.pi 
    hip_y*=-180/np.pi 
    knee*=180/np.pi
    ankle_x = -(hip_x+knee)
    ankle_y = -hip_y 
    return hip_y,hip_x,knee,ankle_x,ankle_y
motor_direction = np.array([1, 1, -1, 1, -1,     1, -1, 1, -1, -1])

initial_angle = 90.0
angle_offset = np.array([-2, 2.14, -2.0, 10, 0.,    -6, 2, 0, -8, 0.])

motor_control(np.zeros(10))
motor_output = np.zeros(10)
time.sleep(1.5)
try:
    while True:
        side = input('l or r')
        ux=float(input('x='))
        uy=float(input('y='))
        uz=float(input('z='))
        hip_y,hip_x,knee,ankle_x,ankle_y= torso_based_ik(ux,uy,uz)
        if side=='l':
            motor_output[0:5]=np.array([hip_y,hip_x,knee,ankle_x,ankle_y])
        if side=='r':
            motor_output[5:10]= np.array([hip_y,hip_x,knee,ankle_x,ankle_y])
        if side=='b':
            motor_output = np.array([ hip_y,hip_x,knee,ankle_x,ankle_y, hip_y,hip_x,knee,ankle_x,ankle_y])
        motor_control(motor_output)
        time.sleep(1)
except KeyboardInterrupt:
    pca.reset()
    print('keyboard interrupt, reset pca')

