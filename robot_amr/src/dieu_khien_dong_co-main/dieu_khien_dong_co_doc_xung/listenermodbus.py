#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64
import time
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus.register_write_message import (WriteMultipleRegistersResponse,WriteSingleRegisterResponse)
from pymodbus.payload import BinaryPayloadBuilder, Endian, BinaryPayloadDecoder


v11=0
v22=0

client = ModbusClient(method='rtu', port='/dev/ttyUSB0', stopbits=1, parity='N', baudrate=115200,timeout=0.05)
connection = client.connect()

data11=client.write_registers(0x2E, 600, unit= 0x01)
data12=client.write_register(0x2F, 1200, unit= 0x01)
data13=client.write_register(0x30,0, unit=0x01)
data14=client.write_register(0x7C, 0x96, unit= 0x01)

data21=client.write_registers(0x2E, 600, unit= 0x02)
data22=client.write_register(0x2F, 1200, unit= 0x02)
data23=client.write_register(0x30,0, unit=0x02)
data24=client.write_register(0x7C, 0x96, unit= 0x02)

def callback(data):
    dulieu=data.data
    a=dulieu.split(",")
    v1=a[0]
    v1= float(v1)
    v1 = round(v1,0)
    
    v2=a[1]
    v2=float(v2)
    v2=round(v2,0)

    global v11
    global v22
    v11 = int(v1)
    v22 = int(v2)
  
def gui_dulieu():
   
    #gui gia tri van toc den dong co 1
    builder1 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
    builder1.reset()
    builder1.add_16bit_int(int(v11))
    payload1=builder1.to_registers()


    data13=client.write_register(0x30, payload1[0],unit=0x01)

    #gui gia tri van toc den dong co 2
    builder2 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
    builder2.reset()
    builder2.add_16bit_int(int(v22))
    payload2=builder2.to_registers()
    
   
    data23=client.write_register(0x30, payload2[0],unit=0x02)
       
    if (v11 == 0) and (v22 == 0):
        data13=client.write_register(0x30, 0,unit=0x01)
        data23=client.write_register(0x30, 0,unit=0x02)
    print("done1")
        
def doc_dulieu():   
    
    encoder_left_pub = rospy.Publisher('/right_ticks', Int64, queue_size=10)
    encoder_right_pub = rospy.Publisher('/left_ticks', Int64, queue_size=10)
    # rospy.init_node('encoder', anonymous=True) 
    

    #đọc giá trị xung cho động cơ 1
    readxungdc1 = client.read_holding_registers(0x04,2,unit=0x01)
    r1_1 = str(hex(readxungdc1.registers[int(0)]))
    r1_2 = str(hex(readxungdc1.registers[int(1)]))
    r1_1 = r1_1[2:]
    r1_2 = r1_2[2:]
    if len(r1_2) == 2:
        r1_2 = "0" + "0" + r1_2
    if len(r1_2) == 3:
        r1_2 = "0" + r1_2
    r1_3 = int(r1_1 + r1_2,base=16)
    # r1_3 = str(r1_3)
    r1_3 = Int64(r1_3)
    print("xungdc1 :",r1_3)    
    
    #đọc giá trị xung cho động cơ 2
    readxungdc2 = client.read_holding_registers(0x04,2,unit=0x02)
    r2_1 = str(hex(readxungdc2.registers[int(0)]))
    r2_2 = str(hex(readxungdc2.registers[int(1)]))
    r2_1 = r2_1[2:]
    r2_2 = r2_2[2:]
    if len(r2_2)==2:
        r2_2 = "0" + "0" + r2_2    
    if len(r2_2)==3:
        r2_2 = "0" + r2_2
    r2_3 = int(r2_1 + r2_2,base=16)
    # r2_3 = str(r2_3)
    r2_3 = Int64(r2_3)
    print("xungdc2 :",r2_3)  

    encoder_left_pub.publish(r1_3)
    encoder_right_pub.publish(r2_3)
   

    print("done2")

    
# def listenermodbus():
#     rospy.init_node('listener', anonymous=True)
#     time.sleep(0.01)
#     rospy.Subscriber('modbus', String, callback)
    

   
# start_time = time.time() 
# while (True):
#     listenermodbus()
#     elapsed_time=time.time() - start_time
#     if elapsed_time >= 0.01:
#         gui_dulieu()
#         doc_dulieu()
#         start_time = time.time()
        
def listenermodbus():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('modbus', String, callback)

def main():
    listenermodbus()
    while True:
        gui_dulieu()
        doc_dulieu()
        time.sleep(0.01)

if __name__ == '__main__':
    main()
        


