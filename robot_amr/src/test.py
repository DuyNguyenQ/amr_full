#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist

from pymodbus.client.sync import ModbusSerialClient




twist_angular = 0.0
twist_linear = 0.0
client = ModbusSerialClient(method="rtu", port = "/dev/ttyUSB0", baudrate = 115200,stopbits = 1, parity = "N",timeout = 0.2)
client.connect()

# stop jog
client.write_register(0x7C, 0xD8, unit= 0x01)
client.write_register(0x7C, 0xD8, unit= 0x02)


# accel
client.write_registers(0x2E, 30, unit= 0x01)
client.write_registers(0x2E, 30, unit= 0x02)

# decel
client.write_register(0x2F, 30, unit= 0x01)
client.write_register(0x2F, 30, unit= 0x02)

# vel
client.write_register(0x30,0, unit=0x01)
client.write_register(0x30,0, unit=0x02)

#  start jog
client.write_register(0x7C, 0x96, unit= 0x01)
client.write_register(0x7C, 0x96, unit= 0x02)





def callback_Velocity(data):
    global twist_angular, twist_linear
    twist_linear = data.linear.x
    twist_angular = data.angular.z
    
    
  
def setVelocity():
    global value_banhtrai,value_banhphai, v_phai, v_trai, wheelbase, twist_linear,twist_angular

    if twist_linear == 2.0:
        client.write_register(0x30,2**16 - 1000, unit=0x01)
        client.write_register(0x30,1000, unit=0x02)
    
    elif twist_linear == -2.0:
        client.write_register(0x30,0, unit=0x01)
        client.write_register(0x30,0, unit=0x02)
    
    elif twist_angular == 2.0:
        client.write_register(0x30,200, unit=0x01)
        client.write_register(0x30,200, unit=0x02)
    
    elif twist_angular == -2.0:
        client.write_register(0x30,2**16 - 200, unit=0x01)
        client.write_register(0x30,2**16 - 200, unit=0x02)
        
        
        
        
def talker ():     
    rospy.init_node('talkermodbus11', anonymous=True)
    rate = rospy.Rate(10)
    rospy.loginfo("Publisher Node Started, now publishing messages")
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback_Velocity)
    while not rospy.is_shutdown():

        setVelocity()
     
        rate.sleep()         	
if __name__== '__main__':
    try:
        # client.write_register(0x7C, 0xD8, unit= 0x01)
        # client.write_register(0x7C, 0xD8, unit= 0x02)
        talker()        
    except rospy.ROSInterruptException:
    	pass