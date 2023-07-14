#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import numpy as np
from tf.transformations import quaternion_from_euler
import math

angular_velocity  = None
angular_velocity_x  = None
angular_velocity_y  = None
angular_velocity_z  = None
linear_acceleration = None
linear_acceleration_x =  None
linear_acceleration_y = None
linear_acceleration_z = None
orientation =  None
pub = None

def callback_angular(data):
    global angular_velocity, angular_velocity_x, angular_velocity_y, angular_velocity_z
   
    angular_velocity_x  = data.angular_velocity.z
    angular_velocity_y  = data.angular_velocity.x * (-1.0)
    angular_velocity_z  = data.angular_velocity.y * (-1.0)

    
def callback_linear(data):
    global  linear_acceleration, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z

   
    linear_acceleration_x = data.linear_acceleration.z
    linear_acceleration_y = data.linear_acceleration.x * (-1.0)
    linear_acceleration_z = (data.linear_acceleration.y ) * (-1.0) 
    
def pub_imu():
    global angular_velocity_x, angular_velocity_y, angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z

    
    imu = Imu()
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = "imu"
    imu.angular_velocity.x = angular_velocity_x
    imu.angular_velocity.y = angular_velocity_y
    imu.angular_velocity.z = angular_velocity_z
    imu.linear_acceleration.x = 0
    imu.linear_acceleration.y = 0
    imu.linear_acceleration.z = linear_acceleration_z

    pub.publish(imu)
    


def main():
    global pub,angular_velocity,linear_acceleration, angular_velocity_x, angular_velocity_y, angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z
    rospy.init_node('imu', anonymous=True)
    
    rospy.Subscriber("/camera/accel/sample",Imu,callback_linear)
    rospy.Subscriber("/camera/gyro/sample",Imu,callback_angular)
    
    pub = rospy.Publisher("imu/data_raw", Imu, queue_size= 10)
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
   
        pub_imu()
        

        rate.sleep
          
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
