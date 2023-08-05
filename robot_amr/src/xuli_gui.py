#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64
import time
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
import threading
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.register_write_message import (WriteMultipleRegistersResponse,WriteSingleRegisterResponse)
from pymodbus.payload import BinaryPayloadBuilder, Endian, BinaryPayloadDecoder

v11=0
v22=0
x0 = 0.1
y0 = 0.1
v1 = 0.1     #gia tri gui xuong cho dong co 1
v2 = 0.2     #gia tri gui xuong cho dong co 2
width = 0.617 #khoang cach giua 2 banh xe

client = ModbusClient(method='rtu', port='/dev/ttyUSB0', stopbits=1, parity='N', baudrate=115200, timeout=0.05)
connection = client.connect()

data14=client.write_register(0x7C, 0xD8, unit= 0x01)

data11=client.write_registers(0x2E, 40, unit= 0x01)
data12=client.write_register(0x2F, 80, unit= 0x01)
data13=client.write_register(0x30,0, unit=0x01)

data14=client.write_register(0x7C, 0x96, unit= 0x01)


data24=client.write_register(0x7C, 0xD8, unit= 0x02)

data21=client.write_registers(0x2E, 40, unit= 0x02)
data22=client.write_register(0x2F, 80, unit= 0x02)
data23=client.write_register(0x30,0, unit=0x02)

data24=client.write_register(0x7C, 0x96, unit= 0x02)



def callback(data):

    global v11,v22    
    
    a = round(data.linear.x,1)
    b = round(data.angular.z,1)
    x0= float(b)    
    y0=float(a)

    #chay thang 
    if((x0 >= -0.25) and (x0 <= 0.25) and (y0 > 0)):           
         v1 = (-1)*abs(y0)
         v2 = abs(y0)
        #  print("thang")
         
    #chay lui
    elif ((x0 >= -0.25) and (x0 <= 0.25) and (y0 < 0)):              
        v1 = abs(y0)
        v2 = (-1)*abs(y0)
        # print("lui")   
         
    #re phai tai cho
    elif((y0 >= -0.25) and (y0 <= 0.25) and (x0 > 0)):         
        v1 = (-1)*abs(x0)
        # v2 = abs(x0)*(4/5)
        v2 = (-1)*abs(x0)
        # print("phai")
        
    #re trai tai cho
    elif((y0 >= -0.25) and (y0 <= 0.25) and (x0 <0)):             
        v1 = abs(x0)
        v2 = abs(x0)
        # print("trai")
        
    #chay thang phai
    elif((x0>0.25) and (y0>0.25)): 
        v1=(-1)*abs(y0 + x0*(width/2))
        v2=abs(y0 - x0*(width/2))

        if (abs(v1)>=1):
            v1=-1
            v2=abs(y0 - x0*(width/2))
    #chay lui phai
    elif((x0>0.25) and (y0<-0.25)): 
        v2=(-1)*abs(y0 + x0*(width/2))
        v1=abs(y0 - x0*(width/2))
        if (abs(v2)>=1):
            v2=(-1)*abs(y0 + x0*(width/2))
            v1=(1)
    #chay lui trai
    elif((x0<-0.25) and (y0<-0.25)): 
        v2=(-1)*abs(y0 + x0*(width/2))
        v1=abs(y0 - x0*(width/2))
        if (abs(v2)>=1):
            v2=(-1)*1
            v1=abs(y0 - x0*(width/2))

    #chay thang trai
    elif((x0<-0.25) and (y0>0.25)):   
        v1=(-1)*abs(y0 + x0*(width/2))
        v2=abs(y0 - x0*(width/2))
        if (abs(v2)>=1):
            v1=(-1)*abs(y0 + x0*(width/2))
            v2=1
    #dung

    elif (x0 == 0) and (y0 == 0):
        v1 = 0
        v2 = 0
        # print("dung")      
         

    # xu li van toc cho dong co 1    
    v11=((v1*16)/1)*240        
    # xu li van toc cho dong co 2    
    v22=((v2*16)/1)*240
    
    v11 = round(v11, 0)
    v22 = round(v22, 0)
 
    # print("---")

def guidulieu():
    global v11,v22    
    
    

    # valuek1 = int(round(abs(v11)*150/12000,0))
    # print(valuek1)
    
    # valuek2 = int(round(abs(v22)*150/12000,0))
    # print(valuek1)
    
    # # client.write_register(0x7C, 0xD8, unit= 0x01)
    
    # # client.write_register(0x7C, 0xD8, unit= 0x02)
    # # client.write_registers(0x2E, valuek1, unit= 0x01)
    # client.write_register(0x2F, valuek1, unit= 0x01)
    # client.write_register(0x7C, 0x96, unit= 0x01)
    # # client.write_register(0x7C, 0xD8, unit= 0x01)

    # # data21=client.write_registers(0x2E, valuek2, unit= 0x02)
    # data22=client.write_register(0x2F, valuek2, unit= 0x02)
    # data24=client.write_register(0x7C, 0x96, unit= 0x02)
    
    # Gui gia tri van toc den dong co 1
    builder1 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
    builder1.reset()
    builder1.add_16bit_int(int(v11))
    payload1 = builder1.to_registers()
    data13 = client.write_register(0x30, payload1[0], unit=0x01)

    # Gui gia tri van toc den dong co 2
    builder2 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
    builder2.reset()
    builder2.add_16bit_int(int(v22))
    payload2 = builder2.to_registers()
    data23 = client.write_register(0x30, payload2[0], unit=0x02)

    if v11 == 0 and v22 == 0:
        data13 = client.write_register(0x30, 0, unit=0x01)
        data23 = client.write_register(0x30, 0, unit=0x02)
        
    
    print("done", v11, v22) 
    
    
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


def talker ():     
    rospy.init_node('talkermodbus1', anonymous=True)
    rate = rospy.Rate(1)
    rospy.loginfo("Publisher Node Started, now publishing messages")
    rospy.Subscriber("cmd_vel", Twist, callback)
    while not rospy.is_shutdown():

        guidulieu()
        doc_dulieu()
        rate.sleep()         	
if __name__== '__main__':
    try:
        # client.write_register(0x7C, 0xD8, unit= 0x01)
        # client.write_register(0x7C, 0xD8, unit= 0x02)
        talker()        
    except rospy.ROSInterruptException:
    	pass
