import ctypes
import struct
import threading
import time
from ctypes import *
from typing import OrderedDict

import numpy as np
import serial

lib=ctypes.CDLL(r"./encoding_LK_Can.dll") 

T_LIMIT=100

P_MIN =0
P_MAX =355.99
V_MIN =-30
V_MAX =30
POSITION_ORDER= b"\x02\x02\x0B\x04\x9C\x7E\x03"


# Python representation of the C struct re_val
class ReVal(Structure):
    _fields_ = [("Lh", c_uint64),("Ll", c_uint64)]
class Read(Structure):
    _fields_ = [("P", c_uint16),("V", c_uint16),("I",c_uint16),("T",c_uint8)]
class motor():

    def __init__(self, motor_id, com1='com11',com2='com9', bps1=115200,bps2=115200):
        self.id=motor_id

        self.disable = -4
        self.enable = -3
        self.setzero= -2

        # 设置串口通讯 
        '''
        self.serial_uart = serial.Serial(com1, bps1)
        self.serial_uart.timeout = 0
        self.serial_uart.bytesize = 8
        self.serial_uart.stopbits = 1
        self.serial_uart.parity = "N"

        self.serial_can = serial.Serial(com2, bps2)
        self.serial_can.timeout = 0
        self.serial_can.bytesize = 8
        self.serial_can.stopbits = 1
        self.serial_can.parity = "N"
        '''

        self.data=[]
        self.crc=[]
        self.P=[]
        self.I=[]
        self.V=[]
        self.T=[]

    def motor_enable(self):
        try:
            self.serial_uart.write(struct.pack(">ii", -4,self.enable))
        except:
            print('cant write motor_enable')

    def motor_disable(self):
            self.serial_uart.write(struct.pack(">ii", -4,self.disable))
    '''
    def motor_setzero(self):
        try:
            #self.serial_can.write(struct.pack(">ii", -4,self.setzero))
            self.serial_uart.write(SETZERO_UART)
        except:
            print('cant write motor_setzero')
    '''


    def motor_sent(self,mode,MotorPara):
        #多线程
        threads = []
        t1 = threading.Thread(target=self.motor_read)
        threads.append(t1)

        if mode=='P' :
            t2 = threading.Thread(target=self.motor_Pctrl_can,args=(MotorPara))
        elif  mode=='V' :
            t2 = threading.Thread(target=self.motor_sent)
        elif mode=='T':
            t2 = threading.Thread(target=self.motor_sent)
        else:
            print("motor_sent wrong")

        threads.append(t2)
        for t in threads:
            t.start()

    def motor_control(self, MotorParam):
        if np.ndim(MotorParam)==1:
            Command=MotorParam[0]
            Pdes=MotorParam[1]
            Ddes = MotorParam[2]
            if Command==0:
                if Pdes<0:
                    Pdes=-Pdes
                    dir=1
                else:
                    dir=0
                Pdes=np.min([ np.max([Pdes,P_MIN]),P_MAX])

                lib.packmsg.restype=ReVal
                a=lib.test(1)
                pacmsg=lib.packmsg( c_int(int(Command)),c_bool(dir),c_int(int(self.id)),c_int(int(Ddes)),c_int(int(Pdes*100)) )

                report=pacmsg.Lh.to_bytes(8,byteorder="big")
                return report
     

  
    def  float_to_uint(self, x,  x_min,  x_max, bits):
        span = x_max - x_min
        if(x < x_min):
            x = x_min
        elif(x > x_max) :
            x = x_max
        p=int( (x- x_min)*float((1<<bits)-1)/span  )
        #print(p)
        return p
    def uint_to_float(x_int, x_min,x_max,bits):
        span = x_max - x_min
        return  float(x_int)*span/float((1<<bits)-1) + x_min

    def motor_read(self):
        while True:
            n = self.serial_uart.inWaiting()  # 等待数据的到来，并得到数据的长度
            if n:  # 如果有数�??

                msg = self.serial_uart.read(1)  # 读取n位数�??
                self.data.append(Read.P)
                lib.motorRead.restype=Read
                Read=lib.motorRead( c_longlong(msg) )
                self.P.append(Read.P)
                self.I.append(Read.I)
                self.V.append(Read.V)
                self.T.append(Read.T)
            time.sleep(0.5)
    def motor_save(self):
        np.savetxt(r".\saveData%d.csv"(self.id),self.data)


if __name__ == '__main__':
    Motorlist=[1,2,3]
    MyMotor=[]
    for i in Motorlist:
        MyMotor.append(motor(i))
    
    MotorPara =np.zeros([len(Motorlist),3,T_LIMIT]) #ID,[CMD VALUEX2]

    Mode=0*np.ones(T_LIMIT) #0-P control
    Pdes1 = np.linspace(10, 300, T_LIMIT)
    Pdes2 = 0.8*np.linspace(10, 300, T_LIMIT)
    Ddes=np.ones(T_LIMIT)*256
    V1=np.array([Mode,Pdes1,Ddes])

    V2=np.array([Mode,Pdes2,Ddes])

    MotorPara[0,:,:]=V1
    MotorPara[1,:,:]=V2
    MotorPara[2,:,:]=V1

    for i in range(T_LIMIT):
        for j in range(len(Motorlist)):
            MyMotor[j].motor_control(MotorPara[j,:,i]) 



    print(MyMotor[0].data,'\n',MyMotor[0].crc)






