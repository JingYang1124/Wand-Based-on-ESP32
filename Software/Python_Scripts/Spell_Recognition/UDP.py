# -*- coding: utf-8 -*-
"""
Created on Tue Dec 15 00:14:11 2020

@author: yangj
"""

import socket
import time
from IMUDataProcess import UpdatePositionAndSave

import pandas as pd
import numpy as np
from sklearn.preprocessing import LabelEncoder
from keras.models import load_model


# Wand_HOST = '192.168.43.205' # The local IP when wand connects to the HOT POINT on my phone
Wand_HOST = '192.168.1.101' # The local IP when wand connects to the WIFI
Wand_PORT = 2333
BUFSIZE = 1024
ADDR = (Wand_HOST, Wand_PORT)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.settimeout(60)
#Local_HOST = '192.168.43.172'# The local IP when this PC connects to the HOT POINT on my phone
Local_HOST = '192.168.1.100'# The local IP when this PC connects to the WIFI
Local_PORT = 233
MaxBytes = BUFSIZE * BUFSIZE
server.bind((Local_HOST, Local_PORT))
server.listen(1)


Unity_client,Unity_addr = server.accept()
print(Unity_addr,"Connected")


def encode(train):
    label_encoder = LabelEncoder().fit(train.species)
    classes = list(label_encoder.classes_)# Original classes that corresponds to the labels(numbers) : ['bad','good']
    return classes

def Predict(data):
    predict=model.predict_classes(data)
    print("Predict result is:")
    for i in range(0,data.shape[0]):    
       print(classes[predict[i]])
    return predict

def Message2RawData(str):
    res = list(filter(None,str.split(" ")))
    RawData = [0.0] * len(res)
    index = 0
    for Info in res:
        if index is len(res) - 1:
            Info = (Info[4:-2]) 
            RawData[index] = int(Info)
            break
        RawData[index] = int(Info[4:])
        index += 1
    index = 0
    return RawData


def RawData2Data(RawData):
    index = 0
    Data = [0.0] * len(RawData)
    while(index <=5):
        if index <=2:
            Data[index] = RawData[index]/4000*9.8
        else:
            Data[index] = RawData[index]/32.4
        index += 1
    return Data

def ControllerGet(x,y):
    print(x)
    print(y)
    if(-4000 < x < -1500):#up
        return "UP"
    if(1500 < x < 4000):#down
        return "DOWN"
    if(-4000 < y < -2000):#right
        return "RIGHT"
    if(2000 < y < 4000):#left
        return "LEFT"
    return "MID"
       

count = 0
READ_DATA_SIZE = 1100
ax = [0.0] * READ_DATA_SIZE
ay = [0.0] * READ_DATA_SIZE
az = [0.0] * READ_DATA_SIZE
gx = [0.0] * READ_DATA_SIZE
gy = [0.0] * READ_DATA_SIZE
gz = [0.0] * READ_DATA_SIZE

model = load_model("Spell_model.h5")
train = pd.read_csv('100train.csv')#导入训练集
classes = encode(train)


CurrentState = "CONTROL"

print("Start")
while True:
    
    if CurrentState == "CONTROL": # Means Controller
        udpCliSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udpCliSock.connect(ADDR)
        Stop_Flag = 0
        while True:
            data = Unity_client.recv(MaxBytes)                      # <-----------------  :Receive Msg from Unity
            data_decode = data.decode('utf-8')
            print("Unity sends that", data_decode) 
            if(data_decode == 'SPELL'):
                Stop_Flag = 1
                CurrentState = "SPELL"
                udpCliSock.close()
                break
            
            command_to_wand = "C"
            udpCliSock.send(command_to_wand.encode('utf-8'))        # ----------------->  :Send Msg to Wand
            time.sleep(0.005)
            data, ADDR = udpCliSock.recvfrom(64)                    # <-----------------  :Receive Msg from Wand
            data = data.decode('utf-8')
            Controller = ""
            if "AcX"  in data:
                Rawdata = Message2RawData(data)
                Controller = ControllerGet(Rawdata[0],Rawdata[1])
                print(Controller)
            Unity_client.send(Controller.encode('utf-8'))           # ----------------->  :Send Msg to Unity

     
    if CurrentState == "SPELL": 
        udpCliSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udpCliSock.connect(ADDR)
        print("Start to read")
        count = 0
        command = "start\n"
        udpCliSock.send(command.encode('utf-8'))
        UdpReadCount = 0
        while True:     
            data, ADDR = udpCliSock.recvfrom(1024)
            data = data.decode('utf-8')
            UdpReadCount += 1
            if UdpReadCount > READ_DATA_SIZE*2:
                udpCliSock.close()
                Spell_To_Unity = "None"
                Unity_client.send(Spell_To_Unity.encode('utf-8'))
                CurrentState = "CONTROL"
                break
                
            if "AcX"  in data:
                Rawdata = Message2RawData(data)
                IMUdata = RawData2Data(Rawdata)
                ax[count] = IMUdata[0]
                ay[count] = IMUdata[1]
                az[count] = IMUdata[2]
                gx[count] = IMUdata[3]
                gy[count] = IMUdata[4]
                gz[count] = IMUdata[5]
                count +=1
                if count >= READ_DATA_SIZE:
                    print("End")
                    FinalData_List = UpdatePositionAndSave(ax,ay,az,gx,gy,gz)
                    FinalData_Array = np.array(FinalData_List)
                    FinalData = FinalData_Array.reshape(1,-1) # Reshape the array -> (1,210)
                    print(FinalData.shape)
                    predict=model.predict_classes(FinalData)

                    print("Predict result is:")
                    for i in range(0,FinalData.shape[0]):    
                        print(classes[predict[i]])

                    Spell_To_Unity = str(classes[predict[i]])
                    Unity_client.send(Spell_To_Unity.encode('utf-8'))
                    udpCliSock.close()
                    CurrentState = "CONTROL"
                    break



