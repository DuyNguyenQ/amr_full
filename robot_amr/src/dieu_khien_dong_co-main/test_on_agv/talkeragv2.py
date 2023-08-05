#!/usr/bin/python3

import rospy
from std_msgs.msg import String
import time
from geometry_msgs.msg import Twist


x0 = 0.1
y0 = 0.1
v1 = 0.1     #gia tri gui xuong cho dong co 1
v2 = 0.2     #gia tri gui xuong cho dong co 2
  


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
         
    #re phai 
    elif((y0 >= -0.25) and (y0 <= 0.25) and (x0 > 0)):         
        v1 = (-1)*abs(x0)
        v2 = abs(x0)*(4/5)
        print("phai")
        
    #re trai  
    elif((y0 >= -0.25) and (y0 <= 0.25) and (x0 <0)):             
        v1 = (-1)*(abs(x0))*(4/5)
        v2 = abs(x0)
        print("trai")
        
    #chay thang phai
    elif((x0>0.25) and (y0>0.25) and (abs(y0)>abs(x0))):   
        if (abs(x0)+abs(y0)>=1): 
            v1 = -1
            v2 = 1*(4/5)
        else:   
            v1 = (-1)*abs(y0)
            v2 = abs(y0)*(4/5)
        print("chay thang phai")
          
    elif((x0>0.25) and (y0>0.25) and (abs(y0)==abs(x0))):   
        v1 = (-1)*abs(y0) 
        v2 = abs(y0)*(4/5)
        print("chay thang phai")
    elif((x0>0.25) and (y0>0.25) and (abs(y0)<abs(x0))):
        if (abs(x0)+abs(y0)>=1): 
            v1 = (-1)*1         #
            v2 = 1*(4/5)                  
        else:
            v1 = (-1)*(abs(x0))
            v2 = (abs(x0))*(4/5) 
        print("chay thang phai")
    #chay lui phai
    elif((x0>0.25) and (y0<-0.25) and (abs(x0)>abs(y0))): 
        if (abs(x0)+abs(y0)>=1):
            v1=1*(4/5)
            v2=-1
        else:  
            v1 = (abs(x0))*(4/5)                         
            v2 = (-1)* (abs(x0))  
        print("chay lui phai")
    elif((x0>0.25) and (y0<-0.25) and (abs(x0)==abs(y0))) :        
        v1 = (abs(x0))*(4/5)
        v2 = (-1)*abs(x0)    
        print("chay lui phai")
    elif((x0>0.25) and (y0<-0.25) and (abs(x0)<abs(y0))) : 
        if (abs(x0)+abs(y0)>=1):
            v1=1*(4/5)
            v2=-1
        else:  
            v1 = (abs(y0))*(4/5)                         
            v2 = (-1)* (abs(y0))
        print("chay lui phai")
    #chay lui trai
    elif((x0<-0.25) and (y0<-0.25) and (abs(x0)<abs(y0))): 
        if (abs(x0)+abs(y0)>=1):
            v1=1
            v2=-1*(4/5)
        else:  
            v1 = (abs(y0))                        
            v2 = (-1)* (abs(y0))*(4/5) 
        print("chay lui trai")
    elif((x0<-0.25) and (y0<-0.25) and (abs(x0)==abs(y0))) :        
        v1 = (abs(x0))
        v2 = (-1)*abs(x0)*(4/5)
        print("chay lui trai")

    elif((x0<-0.25) and (y0<-0.25) and (abs(x0)>abs(y0))) : 
        if (abs(x0)+abs(y0)>=1):
            v1=1
            v2=-1*(4/5)
        else:  
            v1 = (abs(x0))                       
            v2 = (-1)* (abs(x0))*(2/3)
        print("chay lui trai")

    #chay thang trai
    elif((x0<-0.25) and (y0>0.25) and (abs(y0)<abs(x0))):   
        if (abs(x0)+abs(y0)>=1): 
            v1 = -1*(4/5)
            v2 = 1
        else:   
            v1 = (-1)*abs(x0)*(4/5)
            v2 = abs(x0)
        print("chay thang trai")

    elif((x0<-0.25) and (y0>0.25) and (abs(y0)==abs(x0))):   
        v1 = (-1)*abs(y0)*(4/5) 
        v2 = abs(y0)
        print("chay thang trai")

    elif((x0<-0.25) and (y0>0.25) and (abs(y0)>abs(x0))):
        if (abs(x0)+abs(y0)>=1): 
            v1 = (-1)*(4/5)        #
            v2 = 1                  
        else:
            v1 = (-1)*(abs(y0))*(4/5)
            v2 = (abs(y0))
        print("chay thang trai")

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
        
    v11=((v1*2000)/1) #xu li van toc cho dong co 1  
    v22=((v2*2000)/1) #xu li van toc cho dong co 2 
       
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
