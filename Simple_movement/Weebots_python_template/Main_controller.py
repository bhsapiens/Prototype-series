"""Prototype_4_overhead controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Robot
import time
import math
from Library_Stoch import *
import numpy as np

PI = math.pi

# =================================== Joint data ============================================
file_name = 'front'
end_data = np.loadtxt('end_effector_data_'+ file_name + '.txt')
# Theta, percent, FLK, FLH, HLK, HLH, FRK, FRH, HRK, HRH
# 0      1          2   3   4    5      6   7   8    9

# Leg geometry
lh = -0.12 		# Hip link length considering -ve axis
lk = -0.12		# Knee link length

Lh = [0, lh]		# Vectorizing: all links are set to zero about -ve Y-axis
Lk = [0, lk]		# Vectorizing: all links are set to zero about -ve Y-axis

# frequency in Hertz
fre = input("Input frequency (Hz)")	# frequency of oscillation in Hz
w = 2*PI*fre		# Hz to sec/rad conversion


# ===================================== Webots initation ======================================
# create the Robot instance.

robot = Robot()

# get the time step of the current world.

timestep = int(robot.getBasicTimeStep())

keys = robot.getKeyboard()
acc = robot.getAccelerometer('ACC')
ori = robot.getInertialUnit('IMU')

Module = ['FL', 'FR', 'HL', 'HR']
for i in range(4):
    # motor initializing
    m_A[i] = robot.getMotor(Module[i] + 'Am')
    m_H[i] = robot.getMotor(Module[i] + 'Hm')
    m_K[i] = robot.getMotor(Module[i] + 'Km')
    # Encoder initializing
    q_A[i] = robot.getPositionSensor(Module[i] + 'Aps')
    q_H[i]= robot.getPositionSensor(Module[i] + 'Hps')
    q_K[i] = robot.getPositionSensor(Module[i] + 'Kps')

    # Contact Force sensor
    F[i] = robot.getTouchSensor(Module[i] +'TS')
    # Contact trigger sensor
    C[i] = robot.getTouchSensor(Module[i] +'TS')

# GPS
T[0] = robot.getGPS('gps_FL')
T[1] = robot.getGPS('gps_FLb')

# PID loop control
Ka = [50, 0, 0]  # Proportional , Integral, Differential
Kh = [50, 0, 0]
Kk = [50, 0, 0]

for i in range(4):
    m_A[i].setControlPID(Ka[0], 0, 0)
    m_H[i].setControlPID(Kh[0], 0, 0)
    m_K[i].setControlPID(Kk[0], 0, 0)

#============================ Webots enable ====================================
keys.enable(timestep)
acc.enable(timestep)
ori.enable(timestep)

for i in range(2):
    T[i].enable(timestep)
    
for i in range(4):
    
    F[i].enable(timestep)
    C[i].enable(timestep)

    q_A[i].enable(timestep)
    q_H[i].enable(timestep)
    q_K[i].enable(timestep)


# _______________________________________________________________________________________________________
# ======================= Seperating data ================================================
Theta = end_data[:,0]
y_end = [end_data[:,2], end_data[:,2], end_data[:,2], end_data[:,2]]
x_end = [end_data[:,1], end_data[:,1], end_data[:,1], end_data[:,1]]

# ------------------------ Data save ---------------------------------------------------
f_mod = int(f*1000)
file_name = "en_save_"+ str(f_mod) +"_" + str(int(time.time())) +  ".txt"

fl = open(file_name,"w")
fl.close()
fl = open(file_name,"a")

# ================== Variable initation=============================
q = np.zeros([14, 1])
q_3d = np.zeros([14, 1])

theta = [0 , 0, 0 , 0]
theta0 = [0 , 0, 0 , 0]

loop_time = time.time()
begin_time =  time.time()



while 1:
    # User input
    [fre_out, omega] = UserControl(k, fre)
    fre = LowPassFilter(fre, dt, 0, fre_out, 1.25) # need to print  frequency
    w = 2*pi*fre # convert in to radians
    
    # Inverse kinematics and angle transformation of individual legs
    r_rec = [0, 0, 0, 0, 0, 0, 0, 0]

    # Omega to motion
    if omega <0:
        Amp_turn = np.array([0.5, 1, 0.5, 1])
    elif omega > 0:
        Amp_turn = np.array([1, 0.5, 1, 0.5])
    else:
        Amp_turn = np.ones([4,1])

    for i in range(0, 4):
        theta[i] = w * dt +  theta0[i]				# Eular integration
        theta_internal =(theta[i]) % (2*PI)		        # mod(theta, 2 pi)
       
	# First order approximation
        x_ep = np.interp(theta_internal, Theta, x_end)
        y_ep = np.interp(theta_internal, Theta, y_end)

        # Offset control of the legs
        if i in [1,3]:		off_y = 0.0
        else:		off_y = 0

	# Turning stategy
        x_data = Amp_turn[i] * x_ep
        y_data = y_ep

	
        # combining actions
        r = [-x_data, y_data - off_y]

	# Inverse kinematics
        q_IK = IK(Lh, Lk, r)

        r_rec[2*i] = r[0]
        r_rec[2*i + 1] = r[1]

	# Angle transformation between Robot Actuator frame and 
	# calculation assumption
	# [0-Fl 1-Hl 2-Fr 3-Hr]		      # 		Fl Hl Fr Hr
        q_3d[3*i] = q_IK[0] 	      # Hip: 		0  3  6  9
        q_3d[3*i+1] = q_IK[1]	      # Knee: 	        1  4  7  10
        q_3d[3*i+2] = 0 		      # Abduction: 	2  5  8  11
    
    Knee = q_3d[1, 7, 4, 10];
    Hip = q_3d[0, 6, 3, 9];
    Abduction = q_3d[2, 8, 5, 11]
    Spine = q_3d[12, 13]

    # ====================== Out put motor ==========================================
    for i in range(4):
        m1[1].setPosition(Abduction[i])
        m2[2].setPosition(Hip[i])
        m3[3].setPosition(Knee[i])
        
    ms[0].setPosition(Spine[0])
    ms[1].setPosition(Spine[2])
    
    # ----------------------------------------------------------- 
    theta0 = theta
    # ---------------------- Time setting -----------------------
    end_time = time.time()
    dt = end_time  - begin_time
    begin_time =  time.time()
    T  = end_time - loop_time