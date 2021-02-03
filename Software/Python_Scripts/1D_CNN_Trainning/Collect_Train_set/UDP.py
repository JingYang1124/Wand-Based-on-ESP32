# -*- coding: utf-8 -*-
"""
Created on Tue Dec 15 00:14:11 2020

@author: yangj
"""

import socket
import time
from IMUDataProcess import UpdatePositionAndSave

HOST = '192.168.43.205'
PORT = 2333
BUFSIZE = 1024
ADDR = (HOST, PORT)

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



# udpCliSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# udpCliSock.connect(ADDR)
# command = "calibration\n"
# udpCliSock.send(command.encode('utf-8'))
count = 0
READ_DATA_SIZE = 1100
ax = [0.0] * READ_DATA_SIZE
ay = [0.0] * READ_DATA_SIZE
az = [0.0] * READ_DATA_SIZE
gx = [0.0] * READ_DATA_SIZE
gy = [0.0] * READ_DATA_SIZE
gz = [0.0] * READ_DATA_SIZE

oldtime=time.process_time()
SpellName = "LUMOS"
print("Start")
while True:
    command = input('Input:')
    command += '\n'
    if "S" in command: # Means Start

        fax = open('AX.txt','w+')
        fay = open('AY.txt','w+')
        faz = open('AZ.txt','w+')
        fgx = open('GX.txt','w+')
        fgy = open('GY.txt','w+')
        fgz = open('GZ.txt','w+')
        udpCliSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udpCliSock.connect(ADDR)
        print("Start to read")

        count = 0
        command = "start\n"
        udpCliSock.send(command.encode('utf-8'))
        while True:     
            data, ADDR = udpCliSock.recvfrom(1024)
            data = data.decode('utf-8')
            if not data:
                break
            if "AcX"  in data:
                # print(data)
                # print(count)
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
                    for index in range(0,READ_DATA_SIZE):
                        fax.write(str(ax[index]))
                        fax.write('\n')
                        fay.write(str(ay[index]))
                        fay.write('\n')
                        faz.write(str(az[index]))
                        faz.write('\n')
                        fgx.write(str(gx[index]))
                        fgx.write('\n')
                        fgy.write(str(gy[index]))
                        fgy.write('\n')
                        fgz.write(str(gz[index]))
                        fgz.write('\n')
                    fax.close()
                    fay.close()
                    faz.close()
                    fgx.close()
                    fgy.close()
                    fgz.close()
                    UpdatePositionAndSave(SpellName,ax,ay,az,gx,gy,gz)
                    udpCliSock.close()
                    break



newtime=time.process_time()         
print (u'数据处理时间：%s s'%(newtime-oldtime))            
udpCliSock.close()