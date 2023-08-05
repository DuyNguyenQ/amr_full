#!/usr/bin/python3

import rospy
from std_msgs.msg import String
import time
from geometry_msgs.msg import Twist


x0 = 0.1
y0 = 0.1
v1 = 0.1     #gia tri gui xuong cho dong co 1
v2 = 0.2     #gia tri gui xuong cho dong co 2
width = 0.617 #khoang cach giua 2 banh xe


def callback(data):
    pub = rospy.Publisher('modbus', String, queue_size=10)     
    
    a = round(data.linear.x,1)
    b = round(data.angular.z,1)
    x0= float(b)    
    y0=float(a)

    #chay thang 
    if((x0 >= -0.25) and (x0 <= 0.25) and (y0 > 0)):           
         v1 = (-1)*abs(y0)
         v2 = abs(y0)
         print("thang")
         
    #chay lui
    elif ((x0 >= -0.25) and (x0 <= 0.25) and (y0 < 0)):              
        v1 = abs(y0)
        v2 = (-1)*abs(y0)
        print("lui")   
         
    #re phai tai cho
    elif((y0 >= -0.25) and (y0 <= 0.25) and (x0 > 0)):         
        v1 = (-1)*abs(x0)
        # v2 = abs(x0)*(4/5)
        v2 = (-1)*abs(x0)
        print("phai")
        
    #re trai tai cho
    elif((y0 >= -0.25) and (y0 <= 0.25) and (x0 <0)):             
        v1 = abs(x0)
        v2 = abs(x0)
        print("trai")
        
    #chay thang phai
    elif(x0>0.25) and (y0>0.25) : 
        v1=(-1)*abs(y0 + x0*(width/2))
        v2=abs(y0 - x0*(width/2))

        if (abs(v1)>=1):
            v1=-1
            v2=abs(y0 - x0*(width/2))
    #chay lui phai
    elif((x0>0.25) and (y0<-0.25)): 
        v1=abs(y0 + x0*(width/2))
        v2=(-1)*abs(y0 - x0*(width/2))
        if (abs(v2)>=1):
            v1=abs(y0 + x0*(width/2))
            v2=(-1)
    #chay lui trai
    elif((x0<-0.25) and (y0<-0.25)): 
        v1=abs(y0 + x0*(width/2))
        v2=(-1)*abs(y0 - x0*(width/2))
        if (abs(v1)>=1):
            v1=1
            v2=(-1)*abs(y0 - x0*(width/2))

    #chay thang trai
    elif((x0<-0.25) and (y0>0.25)):   
        v1=(-1)*abs(y0 + x0*(width/2))
        v2=abs(y0 - x0*(width/2))
        if (abs(v2)>=1):
            v1=(-1)*abs(y0 + x0*(width/2))
            v2=1


    #dung
    else:
        v1 = 0
        v2 = 0


    if (x0 == 0) and (y0 == 0):
        v1 = 0
        v2 = 0
        print("dung")
       
       
    global v11
    global v22

    # xu li van toc cho dong co 1    
    v11=((v1*25)/1)*240 
        
    # xu li van toc cho dong co 2    
    v22=((v2*25)/1)*240
       
    v11 = str(v11)
    v22 = str(v22)

    giatrigui = v11 + "," + v22  
    print("gia tri gui la:",giatrigui)
    print(x0,y0)
    print("---")
    pub.publish(giatrigui)  
    
    


        
def talker ():
    
    pub = rospy.Publisher('modbus', String, queue_size=10)
    rospy.init_node('talkermodbus', anonymous=True)
    rate = rospy.Rate(1)
    rospy.loginfo("Publisher Node Started, now publishing messages")
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

         	
if __name__== '__main__':
    try:
        talker()        
    except rospy.ROSInterruptException:
    	pass
