from __future__ import division
import sys
import sysv_ipc 
sys.path.append('.')
import RTIMU
import os.path
import time
from smbus import SMBus
import Adafruit_PCA9685
import math
from SherControl_Scripts_global import Inverse_Kinamatics as IK
from SherControl_Scripts_global import Jacobian_Inv as JInv
from SherControl_Scripts_global import *
import numpy as np
key=12
PI = math.pi

# Initializing IMU
SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
    print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
print s
imu = RTIMU.RTIMU(s)
print imu
print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
        print("IMU Init Failed")
        #$sys.exit(1)
else:
        print("IMU Init Succeeded")

    # this is a good time to set any fusion parameters

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

#Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm1.set_pwm_freq(50)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2.set_pwm_freq(50)
bus = SMBus(1)

def read_encoder():
	memory_value = memory.read()

	#Putting those values in a lsit
	encoder_str = []
	encoder_str = memory_value.split('\0')
	encoder_str = filter(None , encoder_str)

	#Convert str into list of integr
	encoder_val = []
	encoder_val = map(int, encoder_str)
	                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
	m=1
	en_ordered=[]
	for en in (0,len(encoder_val)):
              en_ordered.append((encoder_val[0])) #-634)*m)
              en_ordered.append((encoder_val[1])) #-561)*-m)
              en_ordered.append((encoder_val[2])) #-510)*m)
              en_ordered.append((encoder_val[3])) #-803)*-m)
              en_ordered.append((encoder_val[4])) #-510)*m)
              en_ordered.append((encoder_val[5])) #-414)*m)
              en_ordered.append((encoder_val[6])) #-510)*m)
              en_ordered.append((encoder_val[7])) #-393)*m)
              en_ordered.append((encoder_val[8])) #-752)*-m)
              en_ordered.append((encoder_val[9])) #-0)*-m)
              en_ordered.append((encoder_val[10])) #-739)*m)
              en_ordered.append((encoder_val[11])) #-194)*-m)
        #print en_ordered
	return en_ordered

memory = sysv_ipc.SharedMemory(key)

n_FRK = 0; n_FRH = 1;  n_FRA = 2
n_FLK = 15; n_FLH = 14; n_FLA = 13
n_HRK = 15; n_HRH = 14; n_HRA = 13
n_HLK = 0; n_HLH = 1; n_HLA = 2

# Joint data
# trot canter bound on_air_3_1 vel_cap_0_1
gait = 'vel_cap_0_1'

#end_data = np.loadtxt('end_effector_data_' + gait +'.txt')
file_name = 'trot_30to30'
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

# Integration time
dt = 0.05
T_run = 20000

# Motor inputs

m_FLK = 0; m_FLH = 1; m_FLA = 2
m_FRK = 3; m_FRH = 4;  m_FRA = 5
m_HLK = 6; m_HLH = 7; m_HLA = 8
m_HRK = 9; m_HRH = 10; m_HRA = 11

PWM = [300, 300, 325, 325, 300, 300, 400, 300, 325, 325, 300, 300]

# Motor referencing
MR = [0,0,0,0,0,0,0,0,0,0,0,0] # [400, 190, 310, 265, 420, 275, 430, 380, 342, 315, 140, 160] Banckup of the last  working value
   # [FLK  FLH  FLA  FRA  FRH  FRK  HLH  HLK  HLA  HRA  HRK  HRH]

''' Old values:
MR[m_FRK] = 460; MR[m_FRH] = 200; MR[m_FRA] = 265
MR[m_FLK] = 180; MR[m_FLH] = 490; MR[m_FLA] = 300
MR[m_HRK] = 500; MR[m_HRH] = 200; MR[m_HRA] = 315
MR[m_HLK] = 250; MR[m_HLH] = 500; MR[m_HLA] = 350'''

MR[m_FRK] = 460; MR[m_FRH] = 200; MR[m_FRA] = 265
MR[m_FLK] = 180; MR[m_FLH] = 490; MR[m_FLA] = 300
MR[m_HRK] = 500; MR[m_HRH] = 180; MR[m_HRA] = 315
MR[m_HLK] = 250; MR[m_HLH] = 490; MR[m_HLA] = 350

'''
MR[m_FRK] = 460; MR[m_FRH] = 200; MR[m_FRA] = 265
MR[m_FLK] = 180; MR[m_FLH] = 460; MR[m_FLA] = 300
MR[m_HRK] = 500; MR[m_HRH] = 200; MR[m_HRA] = 315
MR[m_HLK] = 250; MR[m_HLH] = 490; MR[m_HLA] = 350'''

Theta = end_data[:,0]
Dtheta = Theta[1] - Theta[0]
Theta_min = min(Theta)

y_end = [end_data[:,3], end_data[:,5], end_data[:,7], end_data[:,9]]
x_end = [end_data[:,2], end_data[:,4], end_data[:,6], end_data[:,8]]

flag = 0

f_mod = int(f*1000)
file_name = "en_save_"+ str(f_mod)+ "_" + gait+ "_" + file_name +"_" + str(int(time.time())) +  ".txt"

fl = open(file_name,"w")
fl.close()
fl = open(file_name,"a")
q = np.zeros([12, 1])
q_3d = np.zeros([12, 1])

theta = [0 , 0, 0 , 0]
theta0 = [0 , 0, 0 , 0]

loop_time = time.time()
begin_time =  time.time()
while 1:
    if flag == 0:
	Leg_no = input("Which leg do you wish to operate ?  ( FL(0) / FR(1) / HL(2) / HR(3) / All(4): \n >> ")
	flag = 1;
	T = 0
    if T == 0:
		pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
		pwm1.set_pwm_freq(50)
		pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
		pwm2.set_pwm_freq(50)
		T += 1
		
    elif T == T_run:
		T = 0
    else:
		T += 1

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
    	#ip = int((theta_internal - Theta_min) / Dtheta)
    	#dtheta = theta_internal - Theta[ip-1]			# difference
       
	# First order approximation
  	x_ep = np.interp(theta_internal, Theta, x_end)
	y_ep = np.interp(theta_internal, Theta, y_end) #y_end[i][ip-1] +  (y_end[i][ip] - y_end[i][ip-1]) * dtheta/Dtheta

        # Offset control of the legs
	if i in [1,3]:
		off_y = 0.0
	else:
		off_y = 0

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
	q_3d[3*i] = PI/2 + q_IK[0] 	      # Hip: 		0  3  6  9
        q_3d[3*i+1] = PI/2 - q_IK[1]	      # Knee: 	        1  4  7  10
	q_3d[3*i+2] = 0 		      # Abduction: 	2  5  8  11


    # Connection/ Transformation between input Data format and Motor input format
    # [Fl Hl Fr Hr] >>> [Fl Fr Hl Hr]
    # Fl > 0 1 2 <-----> 1 0 2  < FL
    # Hl > 3 4 5 -----> 
    # Fr > 6 7 8 ----->
    # Hr > 9 10 11 --->
    qM = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #    [FLK,     FLH,     FLA,     FRA,     FRH,     FRK,     HLH,     HLK,     HLA,     HRA,      HRK,      HRH]
    qMotor = np.array(qM)*180/PI
    
    qM[m_FLK] = q_3d[1]; qM[m_FLH] = q_3d[0]; qM[m_FLA] = q_3d[2]
    qM[m_FRK] = q_3d[7]; qM[m_FRH] = 1*q_3d[6]; qM[m_FRA] = q_3d[8]
    qM[m_HLK] = q_3d[4]; qM[m_HLH] = 1*q_3d[3]; qM[m_HLA] = q_3d[5]
    qM[m_HRK] = q_3d[10]; qM[m_HRH] = 1*q_3d[9]; qM[m_HRA] = q_3d[11]
    

    for i in range(0, 12):
	 		if i in [m_FRA, m_HLA, m_HLK, m_FLK]:  PWM[i] = int(MR[i] + qM[i] * 457 / PI)
			elif i in[m_FLA, m_HRA, m_HRK, m_FRK]: PWM[i] = int(MR[i] - qM[i] * 457 / PI)
	 		elif i in [m_FRH,m_HRH]:               PWM[i] = int(MR[i] + 2 * qM[i] * 160 / PI)
			else:           	        	PWM[i] = int(MR[i] - 2 * qM[i] * 160 / PI)

    if Leg_no == 4 or Leg_no == 3: pwm2.set_pwm(n_HRH, 0, PWM[m_HRH])
    if Leg_no == 4 or Leg_no == 3: pwm2.set_pwm(n_HRK, 0, PWM[m_HRK])
    
    if Leg_no == 4 or Leg_no == 2: pwm2.set_pwm(n_HLH, 0, PWM[m_HLH])
    if Leg_no == 4 or Leg_no == 2: pwm2.set_pwm(n_HLK, 0, PWM[m_HLK])
    
    if Leg_no == 4 or Leg_no == 0: pwm1.set_pwm(n_FLK, 0, PWM[m_FLK])
    if Leg_no == 4 or Leg_no == 0: pwm1.set_pwm(n_FLH, 0, PWM[m_FLH])
    
    if Leg_no == 4 or Leg_no == 1: pwm1.set_pwm(n_FRH, 0, PWM[m_FRH])
    if Leg_no == 4 or Leg_no == 1: pwm1.set_pwm(n_FRK, 0, PWM[m_FRK])

    theta0 = theta
    end_time = time.time()
    dt = end_time  - begin_time
    T  = end_time - loop_time
    #print PWM, dt

    begin_time =  time.time()
    observation=read_encoder()
fl.close()
 
