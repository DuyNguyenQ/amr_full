#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry

position_x = None
position_y = None
orientation_z = None
pub_pose = None
vitri = None
def callback_gps(data):
    global position_x,position_y, orientation_z, vitri
    position_x = data.pose.pose
    # position_y = data.pose.pose.position.y
    # orientation_z = data.pose.pose.orientation.z
    vitri = data.pose.covariance
    
def talk_gps():
    global pub_pose, position_y, position_x, orientation_z, vitri
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()

    odom.header.frame_id = "base_footprint"
    odom.pose.covariance = vitri
    odom.pose.pose = position_x
    # odom.pose.pose.position.y = position_y
    # odom.pose.pose.position.z = 0
    # odom.pose.pose.orientation.x = 0
    # odom.pose.pose.orientation.y = 0
    # odom.pose.pose.orientation.z = orientation_z
    if position_x  is not None:
        pub_pose.publish(odom)


def main():
    global vitri,pub_pose

    # Launch ROS and create a node
    rospy.init_node("gps", anonymous=True)
    
    # Subscribe to ROS topics

    rospy.Subscriber("/camera/odom/sample", Odometry, callback_gps)
    pub_pose = rospy.Publisher("gps/data", Odometry, queue_size=10)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
     
        talk_gps()

       
        rate.sleep()
          
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass    