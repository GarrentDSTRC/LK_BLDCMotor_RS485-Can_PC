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
VCI_USBCAN2 = 4
STATUS_OK = 1



# Python representation of the C struct re_val
class ReVal(Structure):
    _fields_ = [("Lh", c_uint64),("Ll", c_uint64)]
class Read(Structure):
    _fields_ = [("P", c_uint16),("V", c_uint16),("I",c_uint16),("T",c_uint8)]


# Python for ControlCan.dll     
class VCI_INIT_CONFIG(Structure):  
    _fields_ = [("AccCode", c_uint),
                ("AccMask", c_uint),
                ("Reserved", c_uint),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)
                ]  
class VCI_CAN_OBJ(Structure):  
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte),
                ("Data", c_ubyte*8),
                ("Reserved", c_ubyte*3)
                ] 
 
CanDLLName = './ControlCAN.dll' #把DLL放到对应的目录下
canDLL = windll.LoadLibrary(r'./CANAnalysis/ControlCAN.dll')
#Linux系统下使用下面语句，编译命令：python3 python3.8.0.py
#canDLL = cdll.LoadLibrary('./libcontrolcan.so')

print(CanDLLName)
 
ret = canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
if ret == STATUS_OK:
    print('调用 VCI_OpenDevice成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_OpenDevice出错\r\n')
#初始0通道
vci_initconfig = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0,
                                0, 0x03, 0x1C, 0)#波特率125k，正常模式
ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))
if ret == STATUS_OK:
    print('调用 VCI_InitCAN1成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_InitCAN1出错\r\n')

ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
if ret == STATUS_OK:
    print('调用 VCI_StartCAN1成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_StartCAN1出错\r\n')

#初始1通道
ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 1, byref(vci_initconfig))
if ret == STATUS_OK:
    print('调用 VCI_InitCAN2 成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_InitCAN2 出错\r\n')

ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 1)
if ret == STATUS_OK:
    print('调用 VCI_StartCAN2 成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_StartCAN2 出错\r\n')

# Python for ControlCan.dll END    

    
class VCI_CAN_OBJ_ARRAY(Structure):
    _fields_ = [('SIZE', ctypes.c_uint16), ('STRUCT_ARRAY', ctypes.POINTER(VCI_CAN_OBJ))]

    def __init__(self,num_of_structs):
                                                                 #这个括号不能少
        self.STRUCT_ARRAY = ctypes.cast((VCI_CAN_OBJ * num_of_structs)(),ctypes.POINTER(VCI_CAN_OBJ))#结构体数组
        self.SIZE = num_of_structs#结构体长度
        self.ADDR = self.STRUCT_ARRAY[0]#结构体数组地址  byref()转c地址

    

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
                #a=lib.test(1)
                pacmsg=lib.packmsg( c_int(int(Command)),c_bool(dir),c_int(int(self.id)),c_int(int(Ddes)),c_int(int(Pdes*100)) )

                report=pacmsg.Lh.to_bytes(8,byteorder="big")

                #HeadFlag=b'0x140'+str.encode("{:02x}".format(self.id))
                HeadFlag=320+self.id

                #通道1发送数据

                a = (ctypes.c_ubyte*8)(*(report))

                ubyte_3array = c_ubyte*3
                b = ubyte_3array(0, 0 , 0)
                vci_can_obj = VCI_CAN_OBJ(c_uint(HeadFlag), 0, 0, 1, 0, 0,  8, a, b)#单次发送
                
                ret = canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
                if ret == STATUS_OK:
                    print('CAN1通道发送成功\r\n')
                if ret != STATUS_OK:
                    print('CAN1通道发送失败\r\n')

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
        rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(2500)#结构体数组

        ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 1, byref(rx_vci_can_obj.ADDR), 2500, 0)
        #print(ret)
        while ret <= 0:#如果没有接收到数据，一直循环查询接收。
                ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 1, byref(rx_vci_can_obj.ADDR), 2500, 0)
        if ret > 0:#接收到一帧数据
            print('CAN2通道接收成功\r\n')
            print('ID：')
            print(rx_vci_can_obj.STRUCT_ARRAY[0].ID)
            print('DataLen：')
            print(rx_vci_can_obj.STRUCT_ARRAY[0].DataLen)
            print('Data：')
            print(list(rx_vci_can_obj.STRUCT_ARRAY[0].Data))
            # data=0
            # for i in rx_vci_can_obj.STRUCT_ARRAY[0].Data:
            #     data=(data<<8)+i
            # data=struct.pack(">Q",data)
            data=int.from_bytes(rx_vci_can_obj.STRUCT_ARRAY[0].Data,"big")

            lib.motorRead.restype=Read
            read=lib.motorRead( c_longlong(data) )
            self.crc.append(rx_vci_can_obj.STRUCT_ARRAY[0].ID-320)
            self.P.append(read.P)
            self.I.append(read.I)
            self.V.append(read.V)
            self.T.append(read.T)

    def motor_save(self):
        data=np.array([self.crc,self.P,self.I,self.V,self.T])
        np.savetxt(r".\saveData_Motor%d_PIVT.csv"%(self.id),data)


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
            MyMotor[j].motor_read() 

    for j in range(len(Motorlist)):
        MyMotor[j].motor_save()


    #关闭
    canDLL.VCI_CloseDevice(VCI_USBCAN2, 0) 



