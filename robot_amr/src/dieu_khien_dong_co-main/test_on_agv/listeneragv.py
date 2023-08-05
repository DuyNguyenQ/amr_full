#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
import time
import threading
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
#from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus.register_write_message import (WriteMultipleRegistersResponse,WriteSingleRegisterResponse)
from pymodbus.payload import BinaryPayloadBuilder, Endian, BinaryPayloadDecoder

v11=0
v22=0

client = ModbusClient(method='rtu', port='/dev/ttyUSB0',  baudrate=19200,stopbits=1, parity="E", timeout=0.05)
connection = client.connect()

# chon mode 3 wire
client.write_registers(0x1041,0x0001, unit = 0x0001)
client.write_registers(0x1041,0x0001, unit = 0X0002)
# #config lower
client.write_registers(0x018D,0x0001, unit = 0x0001)
client.write_registers(0x018D,0x0001, unit = 0X0002)
# # operation data no 2
client.write_registers(0x007D,0x0002, unit = 0x0001)
client.write_registers(0x007D,0x0002, unit = 0X0002)



#set accel, decel
client.write_registers(0x0605,0x0A, unit = 0x0001)
client.write_registers(0x0605,0x0A, unit = 0X0002)


client.write_registers(0x0685,0x14, unit = 0x0001)
client.write_registers(0x0685,0x14, unit = 0X0002)

# # set speed
client.write_registers(0x0485,0x0000, unit = 0x0001)
client.write_registers(0x0485,0x0000, unit = 0X0002)

##start
client.write_registers(0x7D,0x1A, unit = 0X0001)
client.write_registers(0x7D,0x1A, unit = 0X0002)


def callback(data):
    # rospy.loginfo("RECEIVED DATA: %s", data.data) 
    dulieu=data.data
    a=dulieu.split(",")
    # print("du lieu nhan duoc la: ", a)
    v1=a[0]
    v1= float(v1)
    v1 = round(v1,0)
    # print("v1: ",v1)
    
    v2=a[1]
    v2=float(v2)
    v2=round(v2,0)
    # print("v2: ",v2)

    # print(dulieu)
    global v11
    global v22
    v11 = int(v1)
    v22 = int(v2)
    # print("from callback:",v11, v22)
  
def guidulieu():    
    
    global v11
    global v22
    
    
   
        
    if (v11 < 0) and (v22 > 0):
        client.write_registers(0x0485,abs(v11), unit = 0x0001)
        client.write_registers(0x0485,abs(v22), unit = 0x0002)
        client.write_registers(0x7D,0x1A, unit = 0x0001)
        client.write_registers(0x7D,0x3A, unit = 0x0002)
        print("v11<0 and v22>0")
        
    if (v11 > 0) and (v22 <  0):
        client.write_registers(0x0485,abs(v11), unit = 0x0001)       
        client.write_registers(0x0485,abs(v22), unit = 0x0002)
        client.write_registers(0x7D,0x3A, unit = 0x0001)
        client.write_registers(0x7D,0x1A, unit = 0x0002)
        print("v11>0 and v22<0")
        
    if (v11==0) and (v22==0):
        client.write_registers(0x0485,0x0000, unit = 0X0001)
        client.write_registers(0x0485,0x0000, unit = 0X0002)  
        print("v11==0 and v22==0")
    
    print("done",v11,v22)
    print("------")
   
# def listenermodbus():
#     rospy.init_node('listenerdc1', anonymous=True)
#     time.sleep(0.3)
#     rospy.Subscriber('modbus', String, callback)
#     # rospy.spin()

# start_time = time.time()
# while (True):

#     listenermodbus()
#     elapsed_time=time.time() - start_time
    
#     if elapsed_time >= 0.3:
        
#         guidulieu()
#         start_time = time.time()

def listenermodbus():
    rospy.init_node('listenerdc1', anonymous=True)
    rospy.Subscriber('modbus', String, callback)

def main():
    listenermodbus()
    while True:
        guidulieu()
        time.sleep(0.1)

if __name__ == '__main__':
    main()

