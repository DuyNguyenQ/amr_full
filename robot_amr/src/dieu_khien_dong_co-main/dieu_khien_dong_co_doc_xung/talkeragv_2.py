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
    # rate = rospy.Rate(100)
    # rospy.loginfo("RECEIVED DATA: %s", data.data)
    # dulieu=data.data
    # print("du lieu nhan duoc la: ", dulieu)
    
    
    
    a = round(data.linear.x,1)
    b = round(data.angular.z,1)
    
    #Xu li du lieu
   
    x0= float(b)
    
    # print("x0: ",x0)
    
    
    y0=float(a)
   
    # print("y0: ",y0)
    
    #chay thang chay lui
    if ((x0 >= -0.25) and (x0 <= 0.25)): 
        if (y0 > 0):           #chay thang
            v1 = (-1)*abs(y0)
            v2 = abs(y0)
        elif (y0 < 0):        #chay lui
            v1 = abs(y0)
            v2 = (-1)*abs(y0)
        else:
            v1 = 0
            v2 = 0
            
    
 
    #re phai va re trai tai cho
    if ((y0 >= -0.25) and (y0 <= 0.25)):
        if (x0 > 0):           #re phai tai cho
            v1 = (-1)*abs(x0)
            v2 = (-1)*abs(x0)
        elif (x0 < 0):        #re trai tai cho
            v1 = abs(x0)
            v2 = abs(x0)
        else:
            v1 = 0
            v2 = 0
            
         
    

   
    #chay thang phai
    if (x0>0.25) and (y0>0.25):
        if (abs(y0)>abs(x0)):
            v1 = (-1)*abs(y0)
            v2 = abs(y0)/2
        elif (abs(x0) == abs(y0)):
            v1 = (-1)*abs(y0) 
            v2 = abs(y0)/2
        elif (abs(y0)<abs(x0)):
            v1 = (-1)*(abs(x0)+abs(y0))            
            if (abs(x0)+abs(y0)>1):  #
                v1 = (-1)*1         #
                v2 = abs(x0)/2
                
            if (abs(x0)+abs(y0)>1):  #
                v1 = (-1)*1         #
                v2 = 1/2
            else:
                v2 = (abs(x0)+abs(y0))/2 
        else:
            v1 = 0
            v2 = 0
    
    #chay lui phai
    if (x0>0.25) and (y0<-0.25):
        if (abs(x0)>abs(y0)):                      
            
            v2 = (-1)* (abs(x0)+abs(y0))
            v1 = (abs(x0)+abs(y0))/2
            
                
            if (abs(x0)+abs(y0))>1:  #
                v2 = -1  
                v1 = 1/2
            else:                
               v1 = (abs(x0)+abs(y0))/2 
               v2 = (-1)* (abs(x0)+abs(y0))
              
            
        elif (abs(x0) == abs(y0)):          
            
            v2 = (-1)*abs(x0) 
            v1 = (abs(x0))/2
        elif (abs(x0) < abs(y0)):
            
            v2 = (-1)*abs(y0)
            v1 = (abs(y0))/2
        else:
            v1 = 0
            v2 = 0
    
    #chay lui trai
    if (x0<-0.25) and (y0<-0.25):
        if (abs(x0)<abs(y0)): 
            v1 = abs(y0)         
            v2 = (-1)*(abs(y0))/2
                       
            
            
        elif ((x0)==(y0)):
            
            v1 = abs(x0)          
            v2 = (-1)*(abs(x0))/2
            
            
            
        elif (abs(x0)>abs(y0)):
            v1 = (abs(x0)+abs(y0))
            v2= (-1)*((abs(x0)+abs(y0)))/2
            if (abs(x0) + abs(y0))>1 :
              v1=1
              v2=(-1)/2         
               
        else:
            v1 = 0
            v2 = 0
            
        

    #chay thang trai
    if (x0<-0.25) and (y0>0.25):
        if (abs(x0)>abs(y0)):
            v1 = (-1)*(abs(x0)/2)
            v2 = abs(x0)+abs(y0)
            if (abs(x0)+abs(y0)) > 1:  #
                v2 =1                  # 
        elif (abs(x0)==abs(y0)):
            v1 = (-1)*(abs(x0)/2)
            v2 = abs(x0)
        elif (abs(x0)<abs(y0)):
            v1 = (-1)*(abs(y0)-abs(x0))
            v2 = abs(y0)
        else:
            v1 = 0
            v2 = 0
            
            
    if((x0>= -0.25)and (x0<=0.25) and (y0>=-0.25) and (y0<=0.25)):
        v1=0
        v2=0        

    #dung
    if (x0 == 0) and (y0 == 0):
        v1 = 0
        v2 = 0
       
       
    global v11
    global v22
    
    # xu li van toc cho dong co 1
    if (v1 > 0)  and (v1 <= 1):
       v11=((v1*25)/1)*240
    elif (v1 < 0)  and (v1 >= -1):
       v11=((v1*25)/1)*240
    else:
       v11= 0

    # xu li van toc cho dong co 2
    if (v2 > 0)  and (v2 <= 1):
       v22=((v2*25)/1)*240
    elif (v2 < 0) and (v2 >= -1):
       v22=((v2*25)/1)*240
    else:
       v22= 0
      
    # v11=((v1*1000)/1)
    # v22=((v2*1000)/1)
       
    v11 = str(v11)
    v22 = str(v22)

    giatrigui = v11 + "," + v22  
    print("gia tri gui la:",giatrigui)
    pub.publish(giatrigui)  
    
    print(x0,y0)


        
def talker ():
    
    pub = rospy.Publisher('modbus', String, queue_size=1)
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
