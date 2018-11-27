"""Prototype_4_overhead controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Robot
import math
from Library_stoch import *
import numpy as np

PI = math.pi
# =================================== Joint data ============================================
file_name = 'Hind'
end_data = np.loadtxt('end_effector_data_'+ file_name + '.txt')
# Theta, percent, FLK, FLH, HLK, HLH, FRK, FRH, HRK, HRH
# 0      1          2   3   4    5      6   7   8    9

# Leg geometry
lh = -0.12 		# Hip link length considering -ve axis
lk = -0.12		# Knee link length

Lh = [0, lh]		# Vectorizing: all links are set to zero about -ve Y-axis
Lk = [0, lk]		# Vectorizing: all links are set to zero about -ve Y-axis

# frequency in Hertz
fre = 0	# frequency of oscillation in Hz
w = 2*PI*fre		# Hz to sec/rad conversion


# ===================================== Webots initation ======================================
# create the Robot instance.

robot = Robot()

# get the time step of the current world.

timestep = int(robot.getBasicTimeStep())
dt =  timestep * 1e-3;

keys = robot.getKeyboard()

acc = robot.getAccelerometer('accelerometer')
ori = robot.getInertialUnit('IMU')
gyro = robot.getInertialUnit('gyro')

Module = ['FL', 'FR', 'HL', 'HR']
m_H = [0, 0, 0, 0]; q_H = [0, 0, 0, 0]
m_K = [0, 0, 0, 0]; q_K = [0, 0, 0, 0]
T = [0]
for i in range(4):
    # motor initializing
    #m_A[i] = robot.getMotor(Module[i] + 'Am')
    m_H[i] = robot.getMotor(Module[i] + 'Hm')
    m_K[i] = robot.getMotor(Module[i] + 'Km')
    # Encoder initializing
    #q_A[i] = robot.getPositionSensor(Module[i] + 'Aps')
    q_H[i]= robot.getPositionSensor(Module[i] + 'Hps')
    q_K[i] = robot.getPositionSensor(Module[i] + 'Kps')

    # Contact Force sensor
    #F[i] = robot.getTouchSensor(Module[i] +'TS')
    # Contact trigger sensor
    #C[i] = robot.getTouchSensor(Module[i] +'TS')

# GPS
T[0] = robot.getGPS('gps_spine')

# PID loop control
Ka = [20, 0, 0]  # Proportional , Integral, Differential
Kh = [20, 0, 0]
Kk = [20, 0, 0]

for i in range(4):
    #m_A[i].setControlPID(Ka[0], 0, 0)
    m_H[i].setControlPID(Kh[0], 0, 0)
    m_K[i].setControlPID(Kk[0], 0, 0)

#============================ Webots enable ====================================
keys.enable(timestep)
acc.enable(timestep)
ori.enable(timestep)

for i in range(1):
    T[i].enable(timestep)
    
for i in range(4):
    
    #F[i].enable(timestep)
    #C[i].enable(timestep)

    #q_A[i].enable(timestep)
    q_H[i].enable(timestep)
    q_K[i].enable(timestep)


# _______________________________________________________________________________________________________
# ======================= Seperating data ================================================

Theta = np.loadtxt('end_effector_data_theta_'+ file_name + '.txt')
y_end = np.loadtxt('end_effector_data_y_'+ file_name + '.txt')
x_end = np.loadtxt('end_effector_data_x_'+ file_name + '.txt')

Dtheta = Theta[1] - Theta[0]
Theta_min = min(Theta)
Phase = [PI, 0, 0, PI]
# ------------------------ Data save ---------------------------------------------------
#file_name = "en_save_" + str(int(time.time())) +  ".txt"

#fl = open(file_name,"w")
#fl.close()
#fl = open(file_name,"a")

# ================== Variable initation=============================
q = np.zeros([14, 1])
q_3d = np.zeros([14, 1])

theta = [0 , 0, 0 , 0]
theta0 = [0 , 0, 0 , 0]

i_count = 1
while robot.step(timestep) != -1:
    # User input
    k =keys.getKey()
    [fre_out, vel_side, omega] = UserControl(k, fre)
    #omega = 0
    fre = LowPassFilter(fre, dt, 0, fre_out, 1.25) # need to print  frequency
    w = 2*PI*fre # convert in to radians
    
    # Inverse kinematics and angle transformation of individual legs
    r_rec = [0, 0, 0, 0, 0, 0, 0, 0]

    # Omega to motion
    if omega <0:
        Amp_turn = np.array([-0.5, -0.5, 1, 1])
    elif omega > 0:
        Amp_turn = np.array([1, 1, -1, -1])
    else:
        Amp_turn = np.array([1, 1, 1, 1])

    for i in range(4):
         theta[i] = w * dt +  theta0[i]				# Eular integration
         theta_internal =(theta[i] + Phase[i]) % (2*PI)		        # mod(theta, 2 pi)
         ip = int((theta_internal - Theta_min) / Dtheta)
         dtheta = theta_internal - Theta[ip-1]
	 # First order approximation
         x_ep = x_end[ip-1] +  (x_end[ip] - x_end[ip-1]) * dtheta/Dtheta
         y_ep = y_end[ip-1] +  (y_end[ip] - y_end[ip-1]) * dtheta/Dtheta

        # Offset control of the legs
         if i in [1,3]:		off_y = -0.15
         else:		off_y = -0.175

	 # Turning stategy
         x_data = Amp_turn[i]*x_ep
         y_data = y_ep

	
         # combining actions
         r = [0.02*fre*x_data, 0.01*fre*y_data + off_y]
         #r = [0, -0.175+0.35*fre]
	  # Inverse kinematics
         q_IK = Inverse_Kinamatics(Lh, Lk, r)

         r_rec[2*i] = r[0]
         r_rec[2*i + 1] = r[1]

         # Angle transformation between Robot Actuator frame and 
         # calculation assumption
         # [0-Fl 1-Hl 2-Fr 3-Hr]		      # 		Fl Hl Fr Hr
         q_3d[3*i] = q_IK[0] * 1	      # Hip: 		0  3  6  9
         q_3d[3*i+1] = -q_IK[1] * 1	      # Knee: 	        1  4  7  10
         q_3d[3*i+2] = 0 		      # Abduction: 	2  5  8  11
    q_joint = np.array(q_3d)
    Knee = q_joint[[1, 7, 4, 10]];
    Hip = q_joint[[0, 6, 3, 9]];
 #    #Abduction = q_joint[[2, 8, 5, 11]]
 #    #Spine = q_joint[[12, 13]]

    # ====================== Out put motor ==========================================
    for i in range(4):
        #m1[1].setPosition(Abduction[i])
        m_H[i].setPosition(Hip[i][0])
        m_K[i].setPosition(Knee[i][0]) 
    #ms[0].setPosition(Spine[0])
    #ms[1].setPosition(Spine[2])
    print([fre_out, omega,theta_internal, r])
    
    # ----------------------------------------------------------- 
    theta0 = theta
    i_count += 1
