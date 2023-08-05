#!/usr/bin/python3

import rospy
from std_msgs.msg import String
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
set=False

client = ModbusClient(method='rtu', port='/dev/ttyUSB0', stopbits=1, parity='N', baudrate=115200, timeout=0.05)
connection = client.connect()

data11=client.write_registers(0x2E, 50, unit= 0x01)
data12=client.write_register(0x2F, 50, unit= 0x01)
data13=client.write_register(0x30,0, unit=0x01)
data14=client.write_register(0x7C, 0x96, unit= 0x01)

data21=client.write_registers(0x2E, 50, unit= 0x02)
data22=client.write_register(0x2F, 50, unit= 0x02)
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
    else: 
        v1 = 0.0
        v2 = 0.0 

    # if (x0 == 0) and (y0 == 0):
    #     v1 = 0
    #     v2 = 0
    #     # print("dung")      
         

    # xu li van toc cho dong co 1    
    v11=((v1*50)/1)*240         
    # xu li van toc cho dong co 2    
    v22=((v2*50)/1)*240
    
    v11 = round(float(v11), 0)
    v22 = round(float(v22), 0)
 

    



def guidulieu():
    global v11,v22,set
    
    valuek1 = abs(v11)*150/12000
    valuek2 = abs(v22)*150/12000
    
    data14=client.write_register(0x7C, 0xD8, unit= 0x01)
    data11=client.write_registers(0x2E, valuek1, unit= 0x01)
    data12=client.write_register(0x2F, valuek1, unit= 0x01)
    data14=client.write_register(0x7C, 0x96, unit= 0x01)

    data24=client.write_register(0x7C, 0xD8, unit= 0x02)
    data21=client.write_registers(0x2E, valuek2, unit= 0x02)
    data22=client.write_register(0x2F, valuek2, unit= 0x02)
    data24=client.write_register(0x7C, 0x96, unit= 0x02)
        


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

def talker ():     
    rospy.init_node('talkermodbus', anonymous=True)
    rate = rospy.Rate(10)
    rospy.loginfo("Publisher Node Started, now publishing messages")
    rospy.Subscriber("cmd_vel", Twist, callback)
    while not rospy.is_shutdown():
        guidulieu()
        rate.sleep()         	
if __name__== '__main__':
    try:
        talker()        
    except rospy.ROSInterruptException:
    	pass
