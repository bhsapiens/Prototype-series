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
import numpy as np

key=12
PI = math.pi

def Inverse_Kinamatics_polar(Lh, Lk, r):
    # r = Target point for the 2 link system r: 2x1 [a, b]
    # Lh = initial hip orientation. Usually expressed as Lh: 2x1 [0, lh]
    # Lk = initial knee vector. Lk: 2x1 [0, lk]

    l3 = r[0]
    l1 = Lh[1]
    l2 = Lk[1]

    alpha = r[1]
    v1 = (l1 ** 2 + l2 ** 2 - l3 ** 2)
    v2 = (-2 * l1 * l2)

    cInv = v1/v2
    if cInv >1: cInv = 1
    elif cInv <-1: cInV = -1

    q2 = math.acos(cInv)
    q1 = math.asin(l2 * sin(math.pi - q2) / l3) + alpha

    return [q1, q2+q1]

W1= [-0.0503245,  -0.06732678,  1.0, -0.03611052,  1.0, -0.11518157,
-0.11246687, -0.04629093,  0.12832975, -0.06655705,  1.0,  -0.15476286,
 1.0,          0.0516125,   0.0841161,   0.18423271,
     -0.0503245,  -0.06732678,  1.0, -0.03611052,  1.0, -0.11518157,
-0.11246687, -0.04629093,  0.12832975, -0.06655705,  1.0,  -0.15476286,
 1.0,          0.0516125,   0.0841161,   0.18423271]

W1= [-0.0503245,  -0.06732678,  1.0, -0.03611052,  1.0, -0.11518157,
-0.11246687, -0.04629093,  0.12832975, -0.06655705,  1.0,  -0.15476286,
 1.0,          0.0516125,   0.0841161,   0.18423271,  0.12832975, -0.06655705,  1.0,  -0.15476286,
 1.0,          0.0516125,   0.0841161,   0.18423271, -0.0503245,  -0.06732678,  1.0, -0.03611052,  1.0, -0.11518157,
-0.11246687, -0.04629093]

W =W1;
r_off = [0.175, 0]

#kMP[0] = np.cos(theta)
#kMP[1] = np.cos(2*theta)
#kMP[2] = np.sin(theta)
#kMP[3] = np.sin(2*theta)

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
	en_ordered=[]
	for en in (0,len(encoder_val)):
              en_ordered.append((encoder_val[0]))
              en_ordered.append((encoder_val[1]))
              en_ordered.append((encoder_val[2]))
              en_ordered.append((encoder_val[3])) 
              en_ordered.append((encoder_val[4]))
              en_ordered.append((encoder_val[5]))
              en_ordered.append((encoder_val[6]))
              en_ordered.append((encoder_val[7])) 
              en_ordered.append((encoder_val[8]))
              en_ordered.append((encoder_val[9]))
              en_ordered.append((encoder_val[10]))
              en_ordered.append((encoder_val[11]))
        #print en_ordered
	return en_ordered

memory = sysv_ipc.SharedMemory(key)

n_FRK = 0; n_FRH = 1;  n_FRA = 2
n_FLK = 15; n_FLH = 14; n_FLA = 13
n_HRK = 15; n_HRH = 14; n_HRA = 13
n_HLK = 0; n_HLH = 1; n_HLA = 2

# Folder data
Folder = 'Weight_files/'

# Joint data

k_MP = np.loadtxt(Folder + 'kMP.txt')
gait_name = ['LSWalk','DSWalk','Trot','Pace','Bound','TGallop','RGallop','Canter','Pronk','Symmetric','Modified'];
Weight = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
Z_off = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];

for j in range(0,11):
	Weight[j] = np.loadtxt(Folder + 'Weight_'+ gait_name[j] + '.txt')
        Z_off[j] = np.loadtxt(Folder + 'Mean_'+ gait_name[j] + '.txt')
print " 0.LSWalk 1.DSWalk 2.Trot 3.Pace 4.Bound 5.TGallop 6.RGallop 7.Canter 8.Pronk 9.Symmetric 10.Modified"

N_gait = input("Enter the gait: ")
W = Weight[N_gait]
Mean_off =  Z_off[N_gait]
# Theta, percent, FLK, FLH, HLK, HLH, FRK, FRH, HRK, HRH
# 0      1          2   3   4    5      6   7   8    9

# frequency in Hertz
f = input("Input frequency (Hz)")	# frequency of oscillation in Hz
w = 2*PI*f		# Hz to sec/rad conversion

# Integration time
dt = 0.05
T_run = 20000

# Motor inputs

m_FLK = 0; m_FLH = 1; m_FLA = 2
m_FRK = 3; m_FRH = 4;  m_FRA = 5
m_HLK = 6; m_HLH = 7; m_HLA = 8
m_HRK = 9; m_HRH = 10; m_HRA = 11

# PWM 
PWM = [300, 300, 325, 325, 300, 300, 400, 300, 325, 325, 300, 300]

# Motor referencing
MR = [0,0,0,0,0,0,0,0,0,0,0,0]
   # [FLK  FLH  FLA  FRA  FRH  FRK  HLH  HLK  HLA  HRA  HRK  HRH]

MR[m_FRK] = 460; MR[m_FRH] = 225; MR[m_FRA] = 265
MR[m_FLK] = 180; MR[m_FLH] = 490; MR[m_FLA] = 300
MR[m_HRK] = 500; MR[m_HRH] = 190; MR[m_HRA] = 315
MR[m_HLK] = 250; MR[m_HLH] = 470; MR[m_HLA] = 350


Theta = k_MP[:,0]

flag = 0

f_mod = int(f*1000)
file_name = "en_save_"+ str(f_mod)+ "_" + gait_name[N_gait]+ "_" + str(int(time.time())) +  ".txt"

fl = open(file_name,"w")
fl.close()
fl = open(file_name,"a")

q = np.zeros([8, 1])

theta = 0
theta0 = 0

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

    # --------------------------------------------------------------------------------------------------------
    # --------------------------kMP and angle transformation of individual legs----------------
    # --------------------------------------------------------------------------------------------------------
    theta = (w * dt +  theta0)%(2*PI)				# Eular integration
    kMP = [0,0,0,0]

    kMP[0] = np.cos(theta)
    kMP[1] = np.cos(2*theta)
    kMP[2] = np.sin(theta)
    kMP[3] = np.sin(2*theta)

    #for i_kMP in range(0,4):
	#	kMP[i_kMP] = np.interp(theta, Theta, k_MP[:,i_kMP+1])

    for i in range(0, 4):
	r = []
	r[0] = np.dot(np.array(kMP), np.array(W[:,2*i+1])) + r_off[0]
	r[1] = np.dot(np.array(kMP), np.array(W[:,2*i])) + r_off[1]

	q = Inverse_Kinamatics_polar(Lh, Lk, r)

	theta[i] = 2*PI*fre_out * dt +  theta0[i]				# Eular integration
        theta_internal =(theta[i]) % (2*PI)		# mod(theta, 2 pi)
    	ip = int((theta_internal - Theta_min) / Dtheta)
    	dtheta = theta_internal - Theta[ip-1]

	# Angle transformation
	q[2*i+1] = PI/2 + q[0] 	# Hip: 	1 3 5 7
        q[2*i] = PI/2  - q[1]	# Knee:	0 2 4 6

    # Connection/ Transformation between input Data format and Motor input format
    qM = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]    
    qM[m_FLK] = q[0]; qM[m_FLH] = q[1]; 
    qM[m_FRK] = q[4]; qM[m_FRH] = q[5]; 
    qM[m_HLK] = q[2]; qM[m_HLH] = q[3]; 
    qM[m_HRK] = q[6]; qM[m_HRH] = q[7];
    
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
    fl.write(str(T) + "," + str(observation)+ "," + str(q) + "\n")
    # [Time(1)  Encoder_values(24) Motor_input(8)] 
fl.close()

