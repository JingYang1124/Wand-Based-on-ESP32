# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 12:04:59 2020

@author: yangj
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import numpy as np
from scipy.fftpack import fft,ifft
from scipy import signal
import math
from WriteToExcel import WriteDataToTrainSet

READ_DATA_SIZE = 1100


IMU_MEASURE_INTERVAL_TIME = 0.00225 
# EARTH_G = -9.8
K_D2R = 57.2957

Kp = 10 #比例增益控制加速度计/磁强计的收敛速度
Ki = 0.001 #积分增益控制陀螺偏差的收敛速度

exInt = 0
eyInt = 0
ezInt = 0

cut_index_1 = 0
cut_index_2 = 0
FINAL_PRE_NUM_POINTS = 80
FINAL_NUM_POINTS = 70
STD_VALUE = 0.8


after_ax = [0] * READ_DATA_SIZE
after_ay = [0] * READ_DATA_SIZE
after_az = [0] * READ_DATA_SIZE

def butter_lowpass_filtfilt(data):
	b, a = signal.butter(8, 0.02, 'lowpass', analog = False)
	output = signal.filtfilt(b, a, data, axis=0)

	return output

def butter_highpass_filtfilt(data):
	b, a = signal.butter(4, 0.0006, 'highpass', analog = False)
	output = signal.filtfilt(b, a, data, axis=0)

	return output

def NormalizeAndMergeThreeAxis(datax,datay,dataz):
    Merged_Data = [0.0] * (len(datax) + len(datay) + len(dataz))
    for index in range (0,len(datax)):
        Merged_Data[index] = datax[index]
    for index in range (0 + len(datax),len(datax) + len(datay)):
        Merged_Data[index] = datay[index - len(datax)]
    for index in range (0 + len(datax) + len(datay),len(datax) + len(datay) + len(dataz)):
        Merged_Data[index] = dataz[index - len(datax) - len(datay)]
    Max_Value = max(Merged_Data)
    Min_Value = min(Merged_Data)
    Length = Max_Value - Min_Value
    Mid_Value = (Max_Value + Min_Value)/2
    for index in range(0, len(datax) + len(datay) + len(dataz)):
        Merged_Data[index] = (Merged_Data[index] - Mid_Value)/Length
    return Merged_Data
    

def UpdatePositionAndSave(raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz):
    global after_ax
    global after_ay
    global after_az
    global cut_index_1
    global cut_index_2
    calib_offset = [0.0]*6
    IMUdata = [0.0]*6
    px = [0] * READ_DATA_SIZE
    py = [0] * READ_DATA_SIZE
    pz = [0] * READ_DATA_SIZE
    after_ax = butter_lowpass_filtfilt(raw_ax)
    after_ay = butter_lowpass_filtfilt(raw_ay)
    after_az = butter_lowpass_filtfilt(raw_az)
    after_gx = butter_lowpass_filtfilt(raw_gx)
    after_gy = butter_lowpass_filtfilt(raw_gy)
    after_gz = butter_lowpass_filtfilt(raw_gz)
    
    calib_offset[0] = sum(after_ax[:20]) / 20
    calib_offset[1] = sum(after_ay[:20]) / 20
    calib_offset[2] = sum(after_az[:20]) / 20
    calib_offset[3] = sum(after_gx[:20]) / 20
    calib_offset[4] = sum(after_gy[:20]) / 20
    calib_offset[5] = sum(after_gz[:20]) / 20
    
    Wand = IMU(calib_offset, IMU_MEASURE_INTERVAL_TIME)
    Wand.RotationMatrix_Update()
    
    for i in range(0,READ_DATA_SIZE):
        IMUdata[0] = after_ax[i]
        IMUdata[1] = after_ay[i]
        IMUdata[2] = after_az[i]
        IMUdata[3] = after_gx[i]
        IMUdata[4] = after_gy[i]
        IMUdata[5] = after_gz[i]
        Wand.Gyro_Acc_Update(IMUdata)
        Wand.Quaternions_Update()
        Wand.RotationMatrix_Update()
        Wand.Stable_Check(i)
        Wand.Position_Update(calib_offset)
        px[i] = Wand.position[0]
        py[i] = Wand.position[1]
        pz[i] = Wand.position[2]
        
    after_px = butter_highpass_filtfilt(px)
    after_py = butter_highpass_filtfilt(py)
    after_pz = butter_highpass_filtfilt(pz)
    
    interval = int((cut_index_2 - cut_index_1)/FINAL_PRE_NUM_POINTS)
    inter_x = after_px[cut_index_1:cut_index_2][0:cut_index_2 - cut_index_1:interval][:FINAL_PRE_NUM_POINTS]
    inter_y = after_py[cut_index_1:cut_index_2][0:cut_index_2 - cut_index_1:interval][:FINAL_PRE_NUM_POINTS]
    inter_z = after_pz[cut_index_1:cut_index_2][0:cut_index_2 - cut_index_1:interval][:FINAL_PRE_NUM_POINTS]

    final_x = inter_x[int((FINAL_PRE_NUM_POINTS - FINAL_NUM_POINTS)/2):int(-(FINAL_PRE_NUM_POINTS - FINAL_NUM_POINTS)/2)]
    final_y = inter_y[int((FINAL_PRE_NUM_POINTS - FINAL_NUM_POINTS)/2):int(-(FINAL_PRE_NUM_POINTS - FINAL_NUM_POINTS)/2)]
    final_z = inter_z[int((FINAL_PRE_NUM_POINTS - FINAL_NUM_POINTS)/2):int(-(FINAL_PRE_NUM_POINTS - FINAL_NUM_POINTS)/2)]
    
    x = np.linspace(1, READ_DATA_SIZE, num=READ_DATA_SIZE)

    # fig = plt.figure()
    # fig.clf()
    # plt.plot(x, raw_ax,'b')
    # plt.plot(x, after_ax,'r')
    # plt.show()
    
    # fig2 = plt.figure()
    # fig2.clf()
    # plt.plot(x, raw_ay,'b')
    # plt.plot(x, after_ay,'r')
    # plt.show()
    
    # fig3 = plt.figure()
    # fig3.clf()
    # plt.plot(x, raw_az,'b')
    # plt.plot(x, after_az,'r')
    # plt.show()
    
    # fig4 = plt.figure()
    # fig4.clf()
    # plt.plot(x, px,'b')
    # plt.plot(x, after_px,'r')
    # plt.show()
    
    # fig5 = plt.figure()
    # fig5.clf()
    # plt.plot(x, py,'b')
    # plt.plot(x, after_py,'r')
    # plt.show()
    
    # fig6 = plt.figure()
    # fig6.clf()
    # plt.plot(x, pz,'b')
    # plt.plot(x, after_pz,'r')
    # plt.show()
    
    px_max = max(inter_x)
    py_max = max(inter_y)
    pz_max = max(inter_z)
    px_min = min(inter_x)
    py_min = min(inter_y)
    pz_min = min(inter_z)
    
    
    axis_max = 1.1*max(px_max,py_max,pz_max)
    axis_min = 1.1*min(px_min,py_min,pz_min)
    axis_length = axis_max - axis_min
    
    axis_max_x = (px_min + px_max)/2 + axis_length/2
    axis_min_x = (px_min + px_max)/2 - axis_length/2
    
    axis_max_y = (py_min + py_max)/2 + axis_length/2
    axis_min_y = (py_min + py_max)/2 - axis_length/2
    
    axis_max_z = (pz_min + pz_max)/2 + axis_length/2
    axis_min_z = (pz_min + pz_max)/2 - axis_length/2
    
    fig7 = plt.figure()  # 获取当前图
    plt.ion()
    ax = fig7.gca(projection='3d')  # 获取当前轴
    # ax.scatter(px[:int(READ_DATA_SIZE/2)], py[:int(READ_DATA_SIZE/2)], pz[:int(READ_DATA_SIZE/2)], c='r')
    # ax.scatter(px[int(READ_DATA_SIZE/2):READ_DATA_SIZE], py[int(READ_DATA_SIZE/2):READ_DATA_SIZE], pz[int(READ_DATA_SIZE/2):READ_DATA_SIZE], c='y')            
    # ax.scatter(after_px[:int(READ_DATA_SIZE/2)], after_py[:int(READ_DATA_SIZE/2)], after_pz[:int(READ_DATA_SIZE/2)], c='g')
    # ax.scatter(after_px[int(READ_DATA_SIZE/2):READ_DATA_SIZE], after_py[int(READ_DATA_SIZE/2):READ_DATA_SIZE], after_pz[int(READ_DATA_SIZE/2):READ_DATA_SIZE], c='b')  
    ax.scatter(final_x, final_y, final_z, c='y')
    
    ax.set_zlabel('Z')  # 坐标轴
    ax.set_ylabel('Y')
    ax.set_xlabel('X')
    ax.set_xlim((axis_min_x,axis_max_x))
    ax.set_ylim((axis_min_y,axis_max_y))
    ax.set_zlim((axis_min_z,axis_max_z))
    # plt.show()
    plt.pause(3)  # 暂停一段时间，不然画的太快会卡住显示不出来
    plt.close()
    print("Now predict the data")
    return NormalizeAndMergeThreeAxis(final_x,final_y,final_z)
        

class IMU():
    def __init__(self,C_init,Delta_Time):
        self.q = np.array([1.0,0.0,0.0,0.0])
        self.g = np.array([0.0,0.0,0.0])
        self.a = np.array([C_init[0],C_init[1],C_init[2]])
        self.position = np.array([0.0,0.0,0.0])
        self.v = np.array([0.0,0.0,0.0])
        self.C = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        self.Delta_Time = Delta_Time
        self.yaw = 0
        self.pitch = 0;
        self.roll = 0
    def ReInit(self,C_init):
        self.q = np.array([1.0,0.0,0.0,0.0])
        self.g = np.array([0.0,0.0,0.0])
        self.a = np.array([C_init[0],C_init[1],C_init[2]])
        self.position = np.array([0.0,0.0,0.0])
        self.v = np.array([0.0,0.0,0.0])
        self.C = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
    def Gyro_Acc_Update(self,Data):
        global exInt
        global eyInt
        global ezInt
        
        self.a[0] = Data[0]
        self.a[1] = Data[1]
        self.a[2] = Data[2]
        
        # norm = math.sqrt(self.a[0]**2 + self.a[1]**2 + self.a[2]**2)
        # ax = self.a[0]/norm
        # ay = self.a[1]/norm
        # az = self.a[2]/norm
        
        # #估计方向的重力
        # vx = self.C[2][0]
        # vy = self.C[2][1]
        # vz = self.C[2][2] 
        # ex = (ay*vz - az*vy)
        # ey = (az*vx - ax*vz)
        # ez = (ax*vy - ay*vx)
        
        # #积分误差比例积分增益
        # exInt += ex*Ki
        # eyInt += ey*Ki
        # ezInt += ez*Ki
        
        # Data[3] += Kp*ex + exInt
        # Data[4] += Kp*ey + eyInt
        # Data[5] += Kp*ez + ezInt
        
        self.g[0] = Data[3] / K_D2R
        self.g[1] = Data[4] / K_D2R
        self.g[2] = Data[5] / K_D2R  
    def Quaternions_Update(self):
        self.q[0] = self.q[0] + (-self.q[1]*self.g[0] - self.q[2]*self.g[1] - self.q[3]*self.g[2])*self.Delta_Time/2
        self.q[1] = self.q[1] + (self.q[0]*self.g[0] + self.q[2]*self.g[2] - self.q[3]*self.g[1])*self.Delta_Time/2
        self.q[2] = self.q[2] + (self.q[0]*self.g[1] - self.q[1]*self.g[2] + self.q[3]*self.g[0])*self.Delta_Time/2
        self.q[3] = self.q[3] + (self.q[0]*self.g[2] + self.q[1]*self.g[1] - self.q[2]*self.g[0])*self.Delta_Time/2
        norm = math.sqrt(self.q[0]**2 + self.q[1]**2 + self.q[2]**2 + self.q[3]**2)
        self.q = self.q/norm
    def RotationMatrix_Update(self):
        self.C[0][0] = self.q[0]**2 + self.q[1]**2 - self.q[2]**2 - self.q[3]**2
        self.C[0][1] = 2 * (self.q[1] * self.q[2] - self.q[3] * self.q[0])
        self.C[0][2] = 2 * (self.q[1] * self.q[3] + self.q[0] * self.q[2])
        
        self.C[1][0] = 2 * (self.q[1] * self.q[2] + self.q[0] * self.q[3])
        self.C[1][1] = self.q[0]**2 - self.q[1]**2 + self.q[2]**2 - self.q[3]**2
        self.C[1][2] = 2 * (self.q[2] * self.q[3] - self.q[0] * self.q[1])
        
        self.C[2][0] = 2 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])
        self.C[2][1] = 2 * (self.q[2] * self.q[3] + self.q[0] * self.q[1])
        self.C[2][2] = self.q[0]**2 - self.q[1]**2 - self.q[2]**2 + self.q[3]**2
        
        self.pitch = math.asin(-self.C[2][0])*K_D2R
        self.roll = math.atan2(self.C[2][1],self.C[2][2])*K_D2R
        self.yaw = math.atan2(self.C[1][0],self.C[0][0])*K_D2R
        # print("***************************")
        # print("roll is :")
        # print(roll)        
        # print("pitch is :")
        # print(pitch)
        # print("yaw is :")
        # print(yaw)
        # print("***************************")
        
    def Position_Update(self,offset):   
        global flog
        A_to_Earth = np.dot((self.C),np.array([[self.a[0]],[self.a[1]],[self.a[2]]]))
        Actual_A = A_to_Earth - np.array([[offset[0]], [offset[1]], [offset[2]]]) 
        Old_v = self.v
        Delta_v = np.array([Actual_A[0][0],Actual_A[1][0],Actual_A[2][0]]) * self.Delta_Time
        self.v = Old_v + Delta_v
        # print("New_v is:")
        # print(self.v)
        
        Delta_S = (Old_v + self.v) * self.Delta_Time / 2
        self.position = self.position + Delta_S
    def Stable_Check(self, index):
        global after_ax
        global after_ay
        global after_az
        global cut_index_1
        global cut_index_2
        global STD_VALUE
        check_range = 40
        
        if cut_index_2 == 0:
            if index % check_range == 0 and READ_DATA_SIZE - check_range> index >check_range and (((np.std(after_ax[index:index + check_range]) < STD_VALUE and np.std(after_ay[index:index + check_range]) < STD_VALUE and np.std(after_az[index:index + check_range]) < STD_VALUE))or(np.std(after_ax[index - check_range:index]) < STD_VALUE and np.std(after_ay[index - check_range:index]) < STD_VALUE and np.std(after_az[index - check_range:index]) < STD_VALUE)):
                  if index - cut_index_1 > 300:
                      cut_index_2 = index
                  else:
                      cut_index_1 = index
        if index > READ_DATA_SIZE - 300:
            cut_index_1 = 100
            cut_index_2 = 500
            
        


# fax = open("AX.txt",'r')
# fay = open("AY.txt",'r')
# faz = open("AZ.txt",'r')
# fgx = open("GX.txt",'r')
# fgy = open("GY.txt",'r')
# fgz = open("GZ.txt",'r')

# ax = [0.0] * READ_DATA_SIZE
# ay = [0.0] * READ_DATA_SIZE
# az = [0.0] * READ_DATA_SIZE
# gx = [0.0] * READ_DATA_SIZE
# gy = [0.0] * READ_DATA_SIZE
# gz = [0.0] * READ_DATA_SIZE




# for index in range(0,READ_DATA_SIZE):
#     ax[index] = float(fax.readline()[:-1])
#     ay[index] = float(fay.readline()[:-1])
#     az[index] = float(faz.readline()[:-1])
#     gx[index] = float(fgx.readline()[:-1])
#     gy[index] = float(fgy.readline()[:-1])
#     gz[index] = float(fgz.readline()[:-1])

# fax.close()
# fay.close()
# faz.close()
# fgx.close()
# fgy.close()
# fgz.close()



# oldtime=time.process_time()

# UpdatePositionAndSave("STUPIFY",ax,ay,az,gx,gy,gz)

# newtime=time.process_time()         
        

# print (u'数据处理时间：%s s'%(newtime-oldtime))

     