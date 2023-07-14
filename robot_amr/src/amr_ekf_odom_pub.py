#!/usr/bin/env python3
import rospy
import time

from pymodbus.client.sync import ModbusSerialClient
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import rospy



# banh_1: Right ; banh_2: left

odomOld = Odometry()
odomNew = Odometry()
odom_data_pub = None
odom_data_pub_quat = None

#  Ban kinh banh xe sai so cua driver 8%
r = 0.1
PI = 3.141592654
wheelbase = 0.9

#dis khoang hai banh di duoc trong 1s
distance_right = 0
distance_left = 0

# khoang cach truoc do cua hai banh
previous_right = 0
previous_left = 0

# khoang cach hien tai cua hai banh
current_right = 0
current_left = 0

# Vi tri dat ban dau
initialX = 0.0
initialY = 0.0
initialTheta = 0.0

# Khoang cach va goc gioi han
distanceTolerance = 0.1
angleTolerance = 0.1
angleEndTolerance = 0.1
headingError = 0.0
headingErrorEnd = 0.0

# Nhan duoc gia tri diem den
goToWaypoint = False

# toa do hien tai cuar robot va toa do diem den
currentx = 0.0
currenty = 0.0
currenttheta = 0.0
waypointGoaly = 0.0
waypointGoalx = 0.0
waypointGoaltheta = 0.0




#  Khi nhan duoc diem ban dau False --> True
initialPoseRecieved = False

#  Ket noi dong co 
client = ModbusSerialClient(method="rtu", port = "/dev/ttyUSB0", baudrate = 115200,stopbits = 1, parity = "N",timeout = 0.1)
client.connect()



# sub vi tri dat ban dau trong rviz
def set_initial_2d(rvizClick):
    global odomOld, initialPoseRecieved
    #  vi tri cu ban dau se la vi tri dat dau tien
    odomOld.pose.pose.position.x = rvizClick.pose.position.x
    odomOld.pose.pose.position.y = rvizClick.pose.position.y
    odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z
    initialPoseRecieved = True
  

def set_goal_2d(rvizClick):
    global goToWaypoint, waypointGoalx,waypointGoaly,waypointGoaltheta
    waypointGoalx = rvizClick.pose.position.x
    waypointGoaly = rvizClick.pose.position.y
   
    waypointGoaltheta = rvizClick.pose.orientation.z
    goToWaypoint = True

# Tinh toan khoang cach di cua banh trai va phai sau 0.1s
def Calculate_right_left():
    global distance_left, distance_right,current_left,current_right,previous_left,previous_right
     # Đọc giá trị từ thanh ghi encoder position
    
    response1 = client.read_holding_registers(0x0004, count=2, unit=0x01)
    response2 = client.read_holding_registers(0x0004, count=2, unit=0x02)
    
    # doc gia tri thanh ghi trong mang
    register1_1 = hex(response1.registers[0])
    register1_2 = hex(response1.registers[1])
    
    register2_1 = hex(response2.registers[0])
    register2_2 = hex(response2.registers[1])
    

    value_1_1 = int(register1_1, 16)
    value_1_2 = int(register1_2, 16)
    value_2_1 = int(register2_1, 16)
    value_2_2 = int(register2_2, 16)

    # Xu ly so am
    if value_1_1 > 0x7FFF:
        value_1_1 = value_1_1 - 0X10000
    
    if value_2_1 > 0x7FFF:
        value_2_1 = value_2_1 - 0X10000

    # Ghep hai thanh ghi
    result_banh1 = (value_1_1 << 16) | (value_1_2 &0xFFFF)
    result_banh2 = (value_2_1 << 16) | (value_2_2 &0xFFFF)

    # chuyen doi quang duong di duoc
    current_right = (result_banh1/(65535*4)) *2*PI*r
    current_left = (result_banh2/(65535*4)) *(-2)*PI*r
    #  xu ly banh trai de di khoang cach la +
    distance_right = current_right - previous_right
    distance_left = current_left - previous_left
    
    previous_right = current_right
    previous_left = current_left
    # print("Distance banh1, banh 2: ",distance_right,distance_left)
    time.sleep(0.1)

def publish_quat():
    global odomNew, odom_data_pub_quat

    q = quaternion_from_euler(0, 0, odomNew.pose.pose.orientation.z)
    

    quatOdom = Odometry()
    quatOdom.header.stamp = odomNew.header.stamp
    quatOdom.header.frame_id = "odom"
    quatOdom.child_frame_id = "base_footprint"
    quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x
    quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y
    quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z
    quatOdom.pose.pose.orientation.x = q[0]
    quatOdom.pose.pose.orientation.y = q[1]
    quatOdom.pose.pose.orientation.z = q[2]
    quatOdom.pose.pose.orientation.w = q[3]
    
  

    for i in range(36):
        if i == 0 or i == 7 or i == 14:
            quatOdom.pose.covariance[i] = 0.01
        elif i == 21 or i == 28 or i == 35:
            quatOdom.pose.covariance[i] += 0.1
        else:
            quatOdom.pose.covariance[i] = 0

    odom_data_pub_quat.publish(quatOdom)
    

def update_odom():
    global odomNew, odomOld,distance_left,distance_left,distance_right,wheelbase, currentx,currenty,currenttheta
    # tinh goc quay cua robot trong moi chu ky
    cycleAngle = math.asin((distance_right - distance_left)/wheelbase) 
    
    # tinh khoang cach duong di trung binh cua hai banh moi chu ky
    cycleDistance = (distance_right + distance_left)/2

    # Tinh goc trung binh trong moi chu ky
    avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z
    
    if (avgAngle > PI):
        avgAngle = avgAngle - 2*PI
  
    elif (avgAngle < -PI) :
        avgAngle = avgAngle + 2*PI
    
    currentx = odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + math.cos(avgAngle)*cycleDistance
    currenty = odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + math.sin(avgAngle)*cycleDistance
    currenttheta = odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z
    
    # Kiem tra vi tri moi co loi khonge_link
    if math.isnan(odomNew.pose.pose.position.x) or math.isnan(odomNew.pose.pose.position.y) or math.isnan(odomNew.pose.pose.position.z):
        currentx = odomNew.pose.pose.position.x = odomOld.pose.pose.position.x
        currenty = odomNew.pose.pose.position.y = odomOld.pose.pose.position.y
        currenttheta = odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z

    # Make sure theta stays in the correct range
    if odomNew.pose.pose.orientation.z > PI:
        currenttheta = odomNew.pose.pose.orientation.z = odomNew.pose.pose.orientation.z -  2 * PI
    elif odomNew.pose.pose.orientation.z < -PI:
        odomNew.pose.pose.orientation.z = odomNew.pose.pose.orientation.z + 2 * PI

    # Compute the velocity
    odomNew.header.stamp = rospy.Time.now()
   
    # Save the pose data for the next cycle
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z
    odomOld.header.stamp = odomNew.header.stamp
    

    # Publish the odometry message
    odom_data_pub.publish(odomNew)
    

def main():
    global odom_data_pub, odom_data_pub_quat,goToWaypoint,initialPoseRecieved, odomNew, odomOld

    # Set the data fields of the odometry message
    odomNew.header.frame_id = "odom"
    odomNew.child_frame_id = "base_footprint"
    odomNew.pose.pose.position.x = 0
    odomNew.pose.pose.position.y = 0
    odomNew.pose.pose.orientation.z = 0
    
    odomOld.pose.pose.position.x = initialX
    odomOld.pose.pose.position.y = initialY
    odomOld.pose.pose.orientation.z = initialTheta

    # Launch ROS and create a node
    rospy.init_node("amr_ekf_odom_pub")
    
    # Subscribe to ROS topics
    rospy.Subscriber("initial_2d", PoseStamped, set_initial_2d)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, set_goal_2d)
    
    
   
    # Publisher of simple odom message where orientation.z is an euler angle
    odom_data_pub = rospy.Publisher("odom_data_euler", Odometry, queue_size=100)

    # Publisher of full odom message where orientation is quaternion
    odom_data_pub_quat = rospy.Publisher("odom_data_quat", Odometry, queue_size=100)
    

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
       
        if initialPoseRecieved == True:
            # Calculate_right_left()
            update_odom()
            publish_quat()
           
          
            print ("Current (x, y, theta) = ",currentx,",",currenty,",",currenttheta)
            print ("Waypoint (x, y, theta) = ",waypointGoalx,",",waypointGoaly,",",waypointGoaltheta)
            print("------------------------------------------------------------------------")

            rate.sleep()

if __name__ == "__main__":
    main()
   
   



        
