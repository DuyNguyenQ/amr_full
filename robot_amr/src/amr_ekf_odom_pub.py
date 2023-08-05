#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist

from pymodbus.client.sync import ModbusSerialClient
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


distance_right_previous = 0
distance_left_previous = 0

# banh_1: Right ; banh_2: left

odomOld = Odometry()
odomNew = Odometry()
odom_data_pub = None
odom_data_pub_quat = None

#  Ban kinh banh xe sai so cua driver 8%
r = 0.1
PI = 3.141592654
wheelbase = 0.62

#dis khoang hai banh di duoc trong 1s
distance_right = 0
distance_left = 0

# khoang cach truoc do cua hai banh
previous_right = 0.0
previous_left = 0.0
position_start = True


twist_angular = 0.0
twist_linear = 0.0

# khoang cach hien tai cua hai banh
current_right = 0.0
current_left = 0.0

# Vi tri dat ban dau
initialX = 0.0
initialY = 0.0
initialTheta = 0.0


# Nhan duoc gia tri diem den
goToWaypoint = False

# toa do hien tai cuar robot va toa do diem den
currentx = 0.0
currenty = 0.0
currenttheta = 0.0
waypointGoaly = 0.0
waypointGoalx = 0.0
waypointGoaltheta = 0.0

linearr= None
angularr= None


#  Khi nhan duoc diem ban dau False --> True
initialPoseRecieved = False

#  Ket noi dong co 
client = ModbusSerialClient(method="rtu", port = "/dev/ttyUSB0", baudrate = 115200,stopbits = 1, parity = "N",timeout = 0.2)
client.connect()

# stop jog
client.write_register(0x7C, 0xD8, unit= 0x01)
client.write_register(0x7C, 0xD8, unit= 0x02)


# accel
client.write_registers(0x2E, 30, unit= 0x01)
client.write_registers(0x2E, 30, unit= 0x02)

# decel
client.write_register(0x2F, 40, unit= 0x01)
client.write_register(0x2F, 40, unit= 0x02)

# vel
client.write_register(0x30,0, unit=0x01)
client.write_register(0x30,0, unit=0x02)

#  start jog
client.write_register(0x7C, 0x96, unit= 0x01)
client.write_register(0x7C, 0x96, unit= 0x02)

def positionstart():
    
    global previous_left, previous_right, position_start
    if position_start == True:
# xu ly vi tri ban dau cua dong co
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
        previous_right = (result_banh2/(65535*3)) *2*PI*r
        previous_left = (result_banh1/(65535*3)) *(-2)*PI*r
        
        position_start = False

# Sub yeu cau toc do
def callback_Velocity(data):
    global twist_angular, twist_linear
    twist_linear = data.linear.x
    twist_angular = data.angular.z
    
    
  
def setVelocity():
    global value_banhtrai,value_banhphai, v_phai, v_trai, wheelbase, twist_linear,twist_angular

    # if abs(twist_linear) <= 0.05:
    #     v_trai = round(- (wheelbase * twist_angular), 3)
    #     v_phai = round((wheelbase * twist_angular) , 3)       
    
    # elif abs(twist_angular) <= 0.05:
    #     v_trai = round(twist_linear , 3)
    #     v_phai = round(twist_linear , 3)      
        
    # else:
    v_trai = round(twist_linear - (wheelbase * twist_angular)/2, 3)
    v_phai = round(twist_linear + (wheelbase * twist_angular) /2, 3)   
        

    value_banhtrai = round(v_trai * 4800 / 0.6283)
    value_banhphai = round(v_phai * 4800 / 0.6283)
    
    value2=2**16 - abs(value_banhphai)
    value1=2**16 - abs(value_banhtrai)
    

                        

    if value_banhphai >= 0:
        
        client.write_register(0x30,value= abs(value_banhphai ),unit = 0x02)
     

    elif value_banhphai < 0:
        client.write_register(0x30,value=value2,unit = 0x02)
    
    if value_banhtrai <= 0:
        client.write_register(0x30,value=abs(value_banhtrai),unit = 0x01)
    elif value_banhtrai > 0:
        client.write_register(0x30,value=value1,unit = 0x01) 
        
    # elif value_banhtrai==0 and value_banhphai==0:
    #     client.write_register(0x30,value=0,unit = 0x01) 
    #     client.write_register(0x30,value=0,unit = 0x02) 

        
    print(value_banhtrai, value_banhtrai)


    #print(value_banhtrai, value_banhphai)
    
# def set_vel():
#     global linearr,angularr
#     th_1 = True
#     th_2 = True
#     th_3 =  True
#     th_4 = True
    


#     if linearr == 2.0 and th_1 == True:
#         client.write_register(0x30,1200, unit=0x02)
#         client.write_register(0x30,2**16-1200, unit=0x01)
#         th_1 = False
#         th_2 = True
#         th_3 =  True
#         th_4 = True
#     elif linearr == -2.0 and th_2 == True:
#         client.write_register(0x30,0, unit=0x01)
#         client.write_register(0x30,0, unit=0x02)
#         th_1 = True
#         th_2 = False
#         th_3 =  True
#         th_4 = True
#     elif angularr == 2.0 and th_3 == True:
#         client.write_register(0x30,300, unit=0x01)
#         client.write_register(0x30,300, unit=0x02)
#         th_1 = True
#         th_2 = True
#         th_3 =  False
#         th_4 = True
#     elif angularr == -2.0 and th_4 == True:
#         client.write_register(0x30,2**16-300, unit=0x01)
#         client.write_register(0x30,2**16-300, unit=0x02)
#         th_1 = True
#         th_2 = True
#         th_3 =  Truet
#         th_4 = False      


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

    

# Tinh toan khoang cach di cua banh trai va phai sau 0.1st
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
    current_right = (result_banh2/(65535*3)) *2*PI*r
    current_left = (result_banh1/(65535*3)) *(-2)*PI*r
    #  xu ly banh trai de di khoang cach la +
    distance_right = (current_right - previous_right) 
    distance_left = (current_left - previous_left) 
    
    previous_right = current_right
    previous_left = current_left
    
    # print("Distance banh1, banh 2: ",distance_right,distance_left)
 

def publish_quat():
    global odomNew, odom_data_pub_quat

    q = quaternion_from_euler(0, 0, odomNew.pose.pose.orientation.z)
    

    quatOdom = Odometry()
    quatOdom.header.stamp = odomNew.header.stamp
    quatOdom.header.frame_id = "odom"
    quatOdom.child_frame_id = "base_link"
    quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x
    quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y
    quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z
    quatOdom.pose.pose.orientation.x = q[0]
    quatOdom.pose.pose.orientation.y = q[1]
    quatOdom.pose.pose.orientation.z = q[2]
    quatOdom.pose.pose.orientation.w = q[3]
    quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x
    quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y
    quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z
    quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x
    quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y
    quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z
    
  

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
    odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec())
    odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec())
   
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
    odomNew.pose.pose.position.z = 0
    odomNew.pose.pose.orientation.x = 0
    odomNew.pose.pose.orientation.y = 0
    odomNew.twist.twist.linear.x = 0
    odomNew.twist.twist.linear.y = 0
    odomNew.twist.twist.linear.z = 0
    odomNew.twist.twist.angular.x = 0
    odomNew.twist.twist.angular.y = 0
    odomNew.twist.twist.angular.z = 0
    odomOld.pose.pose.position.x = initialX
    odomOld.pose.pose.position.y = initialY
    odomOld.pose.pose.orientation.z = initialTheta
        
    

    # Launch ROS and create a node
    rospy.init_node("amr_ekf_odom_pub")
    
    # Subscribe to ROS topics
    rospy.Subscriber("initial_2d", PoseStamped, set_initial_2d)
    rospy.Subscriber("goal_2d", PoseStamped, set_goal_2d)
    rospy.Subscriber("cmd_vel", Twist, callback_Velocity)

    
    
   
    # Publisher of simple odom messtage where orientation.z is an euler angle
    odom_data_pub = rospy.Publisher("odom_data_euler", Odometry, queue_size=100)

    # Publisher of full odom message where orientation is quaternion
    odom_data_pub_quat = rospy.Publisher("odom_data_quat", Odometry, queue_size=100)
    

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
       
        if initialPoseRecieved == True:
            
            positionstart()
            Calculate_right_left()
            update_odom()
            publish_quat()
            setVelocity()
           
          
            # print ("Current (x, y, theta) = ",currentx,",",currenty,",",currenttheta)
            # print("------------------------------------------------------------------------")

            rate.sleep()


if __name__ == "__main__":
    try:
    
        main()
    except rospy.ROSInterruptException:
    	pass
    
   



        