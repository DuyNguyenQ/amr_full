#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64
import time
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus.register_write_message import (WriteMultipleRegistersResponse,WriteSingleRegisterResponse)
from pymodbus.payload import BinaryPayloadBuilder, Endian, BinaryPayloadDecoder
import numpy as np

v11=0
v22=0


client = ModbusClient(method='rtu', port='/dev/ttyUSB0', stopbits=1, parity='N', baudrate=115200,timeout=0.05)
connection = client.connect()
print(connection)


#gửi tốc độ và đọc giá trị xung ban đầu cho động cơ 1
w1_1 = client.write_register(0x2E, 0x258,unit=0x01)
w1_2 = client.write_register(0x2F, 0x4B0,unit=0x01)
w1_3 = client.write_register(0x30, 0,unit=0x01)
w1_4 = client.write_register(40124,0x96,unit=0x01)

readxungdc1 = client.read_holding_registers(0x04,2,unit=0x01)
r1_1 = str(hex(readxungdc1.registers[int(0)]))
r1_2 = str(hex(readxungdc1.registers[int(1)]))
r1_1 = r1_1[2:]
r1_2 = r1_2[2:]
if len(r1_2) == 3:
    r1_2 = "0" + r1_2
r1_3_1 = int(r1_1 + r1_2,base=16)



# def callback(data):
#     dulieu=data.data
#     a=dulieu.split(",")   
#     v1= round(float(a[0]),0)
#     v2= round(float(a[1]),0)
#     global v11
#     global v22
#     v11 = int(v1)
#     v22 = int(v2)
   
   
   
def gui_doc_du_lieu_dc1():
    
    global r1_3_1
    
    builder1 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
    builder1.reset()
    builder1.add_16bit_int(int(v11))
    payload1 = builder1.to_registers()
    # w1_3 = client.write_register(0x30, payload1[0],unit=0x01)
    w1_3 = client.write_register(0x30, 2400,unit=0x01)

    
    # if (v11==0):
    #     w1_3 = client.write_register(0x30, 0,unit=0x01)
                
    # print("Done gui du lieu dong co 1:",v11)
    
        
    #đọc giá trị xung tiếp theo cho động cơ 1
    readxungdc1 = client.read_holding_registers(0x04,2,unit=0x01)
    r1_1 = str(hex(readxungdc1.registers[int(0)]))
    print("r1_1",r1_1)
    r1_2 = str(hex(readxungdc1.registers[int(1)]))
    print("r1_2",r1_2)
    print("--")
    r1_1 = r1_1[2:]
    print("r1_1",r1_1)
    r1_2 = r1_2[2:]
    print("r1_2",r1_2)
    
    if len(r1_2) == 2:
        r1_2 = "0" + "0" + r1_2
    if len(r1_2) == 3:
        r1_2 = "0" + r1_2
    print("r1_2",r1_2)
    
    r1_3_2 = int(r1_1 + r1_2,base=16)
    print("r1_3_2",r1_3_2)
        
    xungdc1 = abs(abs(r1_3_2) - abs(r1_3_1)) #xung của động cơ 1 sau các khoảng thời gian đọc được
    print("xungdc1 sau mot khoang thoi gian 1 giay la:",xungdc1)           
    r1_3_1 = r1_3_2
 

 
 

          
# def listenermodbus():
#     rospy.init_node('listener', anonymous=True)
#     time.sleep(0.001)
#     # rospy.Subscriber('modbus', String, callback)

# start_time = time.time()
# while (True):
#     listenermodbus()
#     elapsed_time=time.time() - start_time
#     if elapsed_time >= 1:
        
#         gui_doc_du_lieu_dc1()
#         print("------")
#         start_time = time.time()

def listenermodbus():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('modbus', String, callback)

def main():
    listenermodbus()
    while True:
        gui_doc_du_lieu_dc1()
        time.sleep(1)

if __name__ == '__main__':
    main()


