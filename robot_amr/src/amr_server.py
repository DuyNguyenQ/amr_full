#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import queue
from std_msgs.msg import Int32, Bool
import queue
from tf.transformations import quaternion_from_euler


# Tao task nhiem vu
task_queue = queue.Queue()


# Vi tri diem den
waypointGoalx = None 
waypointGoaly = None
waypointGoaltheta = None


# Nut nhan send tu server
send = False

# Nut nhan continue tu server
choice_continue = False

run = False
call = True


# Ham nhan cac vi tri tram
def get_goal_position(choice):
    global waypointGoaltheta, waypointGoalx, waypointGoaly
   
    # Note Theta la radian (-pi -> pi)
    #  (x, y, theta, call, priority)
    # voi call neu 0 --> che doi nhap hang, 1 --> che doi xuat hang, 2 --> comeback tram sac
    
                        #  toa do xuat hang
    mang_toa_do_goal = [(0.0, 0.0, -1.57, 1, 0), (0.0, 0.3, 0, 1, 1), (0.0, 0.4,0, 1, 2), (3.0, 0.0, 0, 1, 3), (4.0, 0.0, 0, 1, 4), (5.0, 0.0, 0, 1, 5),   # 0 --> 5
                        #  (6.0, 0.0, 0, 1, 6), (7.0, 0.0, 0, 1, 7), (8.0, 0.0, 0, 1, 8), (9.0, 0.0, 0, 1, 9), (10.0, 0.0, 0, 1, 10),  # 6 --> 10
                         
                        #  toa do nhap hang 
                        ( 11.0, 0.0, 0.0, 0, 0), ( 12.0, 0.0, 0.0, 0, 1), ( 13.0, 0.0, 0.0, 0, 2), ( 14.0, 0.0, 0.0, 0, 3), ( 15.0, 0.0, 0.0, 0, 4)
    

                                                                                                                                       ]
    # if 0 <= choice <= 10:
    diem_toa_do_goal = mang_toa_do_goal[choice]
        # waypointGoalx, waypointGoaly= diem_toa_do_goal
        # goal.position.x = waypointGoalx
        # goal.position.y = waypointGoaly

    # else:
    #     print("\nLua chon vi tri khong hop le. Vui long nhap lai.\n")
    #     return None

    
    return diem_toa_do_goal

# Ham in cac nhiem vu dang cho trong task
def print_pending_tasks():
    tasks = list(task_queue.queue)
    if len(tasks) > 0:
        print("\nNhiem vu dang cho giai quyet:")
        for task in tasks:
            print("Tram", task[0:3])
    else:
        print("\nKhong co nhiem vu dang cho")

                
# Ham add cac nhiem vu vao task ( add vi tri tram )         
def add_task_to_queue(choice):
    global run, call
    goal = get_goal_position(choice)
    if goal is not None:
        # kiem tra yeu cau da co trong nhiem vu chua
        if goal not in list(task_queue.queue):
            if run == True and goal[3] == 1:
                task_queue.put(goal)
                rospy.loginfo("Vi tri xuat hang da duoc them vao hang cho")
                
            elif call == True and goal[3] == 0:
                task_queue.put(goal)
                rospy.loginfo("Vi tri nhap hang da duoc them vao hang cho")
            elif run == False and call == False:
                task_queue.put(goal)
                
        else:
            
            pass
         
    # Sap xep theo priority        
    sorted_tasks = sorted(list(task_queue.queue), key=lambda x: (x[3] , x[4] ))
    task_queue.queue.clear()
    for task in sorted_tasks:
        task_queue.put(task)
    
    
        
# Ham nhan lan luot cac nhiem vu
def process_next_task():
    # if not task_queue.empty():
        
        #Note: Voi goc quay phai chuyen sang he quaternion 
        goal = task_queue.get()
        q = quaternion_from_euler(0, 0, goal[2])

        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = "map"
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.pose.position.x = goal[0]
        mb_goal.target_pose.pose.position.y = goal[1]
        mb_goal.target_pose.pose.orientation.x = q[0]
        mb_goal.target_pose.pose.orientation.y = q[1]
        mb_goal.target_pose.pose.orientation.z = q[2]
        mb_goal.target_pose.pose.orientation.w = q[3]
      
    
        rospy.loginfo("Sending goal")
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        ac.wait_for_server()
        ac.send_goal(mb_goal)

        ac.wait_for_result()
        

        #  Kiem tra robot da den vi tri tram yeu cau chua
        if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("The robot has arrived at the goal location")
        else:
            rospy.loginfo("The robot failed to reach the goal location for some reason")


#  Nut nhan tram
def call_back_nutnhantram0(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 
   

def call_back_nutnhantram1(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 
    
def call_back_nutnhantram2(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 
   

def call_back_nutnhantram3(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 
    
def call_back_nutnhantram4(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 
   

def call_back_nutnhantram5(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 
    
def call_back_nutnhantram6(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 
    
def call_back_nutnhantram7(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 
   

def call_back_nutnhantram8(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 
    
def call_back_nutnhantram9(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 
   

def call_back_nutnhantram10(data):
    
    user_choice = data.data
    add_task_to_queue (user_choice) 

# Nut nhan continue thuc hien nhiem vu tiep theo    
def call_back_nutnhan_continue(data):
    global choice_continue
    choice_continue = data.data
     
# Nut nhan send thuc thi cac nhiem vu da gui    
def call_back_nutnhansend(data):
    global send
    send = data.data


def main():
    global user_choice, choice_continue, send, run, call
    rospy.init_node('amr_server', anonymous=True)
    rospy.Subscriber("nut_nhan0",Int32,call_back_nutnhantram0)
    rospy.Subscriber("nut_nhan1",Int32,call_back_nutnhantram1)
    rospy.Subscriber("nut_nhan2",Int32,call_back_nutnhantram2)
    rospy.Subscriber("nut_nhan3",Int32,call_back_nutnhantram3)
    rospy.Subscriber("nut_nhan4",Int32,call_back_nutnhantram4)
    rospy.Subscriber("nut_nhan5",Int32,call_back_nutnhantram5)
    rospy.Subscriber("nut_nhan6",Int32,call_back_nutnhantram6)
    rospy.Subscriber("nut_nhan7",Int32,call_back_nutnhantram7)
    rospy.Subscriber("nut_nhan8",Int32,call_back_nutnhantram8)
    rospy.Subscriber("nut_nhan9",Int32,call_back_nutnhantram9)
    rospy.Subscriber("nut_nhan10",Int32,call_back_nutnhantram10)
    rospy.Subscriber("nut_send",Bool,call_back_nutnhansend)
    rospy.Subscriber("nut_continue",Bool,call_back_nutnhan_continue)

    rate = rospy.Rate(30)
    
    
    while not rospy.is_shutdown():
        
        #  Nhap hang
        while call:
            print("\nNhap vi tri ban muon goi Robot den nhap hang?")
            print("\n0 = Sat")
            print("1 = Pallet 1")
            print("2 = Pallet 2")
            print("3 = Khuon")
            print("4 = Thep")
            print("\nEnter Number: ")
            
            while send == False:
                
                if send == True:
                    
                    pass
            send = False
            call = False
            run = True
            
        #  Xuat hang
        while run:
            print("\nNhap vi tri xuat hang ban muon Robot den?")
            print("\n0 = Tram Sac")
            print("1 = Khu Vuc 1")
            print("2 = Khu Vuc 2")
            print("3 = Nha An")
            print("4 = Khu Vuc 3")
            print("5 = Khu Vuc 4")
            print("6 = Van Phong")
            print("7 = May Cat")
            print("8 = May CNC")
            print("9 = Cua chinh")
            print("10 = Cua phu")
            print("\nEnter Number: ")
            
        
       
            while send ==  False:
        
                if send == True:
                    
                    pass
                  
            run = False 
            send = False
            
        # Gui vi tri sac de robot quay ve khi hoan thanh task
        # if not task_queue.empty():
        #     add_task_to_queue(9)   
        
        # Process the tasks in the queue
        while not task_queue.empty():
            print_pending_tasks()
            process_next_task()
            print("\nNhan CONTINUE de thuc hien yeu cau tiep theo?")
            while choice_continue == False:
                
                if choice_continue == True:
                    pass
            
            choice_continue = False    
       
        
        print("\n-------------------------------------------------------------------\n")    
        run = True
        call = True
        user_choice = None
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass