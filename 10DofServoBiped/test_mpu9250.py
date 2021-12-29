import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from ahrs.filters import Madgwick
import numpy as np
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
i2c = busio.I2C(SCL,SDA)
pca=PCA9685(i2c)
pca.frequency=100
servo0=servo.Servo(pca.channels[0],min_pulse=700,max_pulse=2300,actuation_range=180)
hz=50
q_filter = Madgwick(frequency=hz)
mpu = MPU9250(
    address_ak=AK8963_ADDRESS, 
    address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
    address_mpu_slave=None, 
    bus=1,
    gfs=GFS_1000, 
    afs=AFS_8G, 
    mfs=AK8963_BIT_16, 
    mode=AK8963_MODE_C100HZ)
mpu.magScale = [1.06,1,0.9339]
mpu.mbias = [25.64,31.69,-31.04]
mpu.configure()
def q2e(q):
    w=q[0]
    x=q[1]
    y=q[2]
    z=q[3]
    ysqr = y * y
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    Y = np.degrees(np.arcsin(t2))
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))
    return X, Y, Z
T0=0
Qu=np.array([1.,0.,0.,0.])
servo_ang=0
servo0.angle=servo_ang
time.sleep(1.5)
servo_dir=0.5
T1=0
while True:
    Tt=time.time()
    if Tt-T0>1/hz:
        servo_ang+=servo_dir
        servo0.angle=servo_ang
        if servo_ang>170:
            servo_dir=-0.5
        if servo_ang<10:
            servo_dir=0.5

        gyro=np.array(mpu.readGyroscopeMaster())*np.pi/180
        accl=np.array(mpu.readAccelerometerMaster())*9.81
        mage = np.array(mpu.readMagnetometerMaster()+np.array([-18.,-32.,27.]))
        Qu=q_filter.updateIMU(Qu,gyr=gyro,acc=accl)
       # Qu1=q_filter.updateMARG(Qu,gyr=gyro,acc=accl,mag=mage)
        ex,ey,ez = q2e(Qu)
        T0=Tt
    if Tt-T1>0.02:
        T1=Tt
        print(ex,ey,ez)
   

