#!/usr/bin/env python3
import rospy
from pymodbus.client.sync import ModbusSerialClient

from geometry_msgs.msg import  Twist
from pymodbus.client.sync import ModbusSerialClient

# Thong so cai dat
PI = 3.141592654
wheel_base = 0.9


# Gia tri van toc dua vao dong co
value_banhphai = 0
value_banhtrai = 0

# Van toc hai banh
v_trai = 0
v_phai = 0

twist_linear = 0.0
twist_angular = 0.0


#  Ket noi dong co 
client = ModbusSerialClient(method="rtu", port = "/dev/ttyUSB0", baudrate = 115200,stopbits = 1, parity = "N",timeout = 0.1)
client.connect()

#accel
client.write_register(0x2E,value=600,unit = 0x01)
client.write_register(0x2E,value=600,unit = 0x02)
# decel
client.write_register(0x2F,value=600,unit = 0x01)
client.write_register(0x2F,value=600,unit = 0x02)
# speed
client.write_register(0x30,value=0,unit = 0x01)
client.write_register(0x30,value=0,unit = 0x02)
# start jog
client.write_register(0x7C,value=0x96,unit = 0x01)
client.write_register(0x7C,value=0x96,unit = 0x02)

# Sub yeu cau toc do
def callback_Velocity(data):
    global twist_angular, twist_linear
    twist_linear = data.linear.x
    twist_angular = data.angular.z

# Xu ly toc do dong co
def setVelocity():
    global value_banhtrai,value_banhphai, v_phai, v_trai, wheel_base, twist_linear,twist_angular


    v_trai = round(twist_linear - (wheel_base * twist_angular)/2, 3)
    v_phai = round(twist_linear + (wheel_base * twist_angular)/2 , 3)      
    value_banhtrai = round(v_trai * 4800 / 0.6283)
    value_banhphai = round(v_phai * 4800 / 0.6283)
    if value_banhphai < 0:
        client.write_register(0x30,value=2**16 - abs(value_banhphai),unit = 0x01)
    elif value_banhphai >= 0:
        client.write_register(0x30,value=value_banhphai,unit = 0x01)
    
    if value_banhtrai <= 0:
        client.write_register(0x30,value=abs(value_banhtrai),unit = 0x02)
    elif value_banhtrai > 0:
        client.write_register(0x30,value=2**16 - abs(value_banhtrai),unit = 0x02)   
    
def main():
    

    # Launch ROS and create a node
    rospy.init_node("amr_setvelocity", anonymous=True)
    
    # Subscribe to ROS topics

    rospy.Subscriber("cmd_vel", Twist, callback_Velocity)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
     
        setVelocity()
        print("Value (trai, phai) = ", value_banhtrai ,",", value_banhphai)
       
        rate.sleep()
          
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass