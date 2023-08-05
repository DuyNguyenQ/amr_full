#!/usr/bin/python3

import rospy
from std_msgs.msg import String
import time

x0 = 0.0
y0 = 0.0
v1 = 0.0    #gia tri gui xuong cho dong co 1
v2 = 0.0     #gia tri gui xuong cho dong co 2
  


def callback(data):
    pub = rospy.Publisher('manual', String, queue_size=10)
    # rate = rospy.Rate(100)
    rospy.loginfo("RECEIVED DATA: %s", data.data)
    dulieu=data.data
    print("du lieu nhan duoc la: ", dulieu)
    
    #Xu li du lieu
    a=dulieu.split(",")
    print("du lieu nhan duoc la: ", a)
    x0=a[0]
    x0= float(x0)
    x0 = round(x0,1)
    print("x0: ",x0)
    
    y0=a[1]
    y0=float(y0)
    y0=round(y0,1)
    print("y0: ",y0)
    1 
    #chay thang chay lui
    if ((x0 >= -0.25) and (x0<=0.25)): 
        if (y0 > 0):           #chay thang
            v1 = (-1)*abs(y0)
            v2 = abs(y0)
        elif (y0 < 0):        #chay lui
            v1 = abs(y0)
            v2 = (-1)*abs(y0)
        else:
            v1 = 0
            v2 = 0
            
    
 
    #re phai re trai
    if ((y0 >= -0.25) and (y0<=0.25)):
        if (x0 > 0):           #re phai
            v1 = (-1)*abs(x0)
            v2 = (-1)*abs(x0)
        elif (x0 < 0):        #re trai
            v1 = abs(x0)
            v2 = abs(x0)
        else:
            v1 = 0
            v2 = 0

   
    #chay thang phai
    if (x0>0.25) and (y0>0.25):
        if (abs(y0)>abs(x0)):
            v1 = (-1)*abs(y0)
            v2 = abs(y0)-abs(x0)
        elif (abs(x0) == abs(y0)):
            v1 = (-1)*abs(y0) 
            v2 = abs(y0)/2
        elif (abs(y0)<abs(x0)):
            v1 = (-1)*(abs(x0)+abs(y0))            
            if (abs(x0)+abs(y0)>1):  #
                v1 = (-1)*1         #
                v2 = abs(x0)/2
            v2 = abs(x0)/2
        else:
            v1 = 0
            v2 = 0
    
    #chay lui phai
    if (x0>0.25) and (y0<-0.25):
        if (abs(x0)>abs(y0)):
            v1 = abs(x0)+abs(y0)
            if (abs(x0)+abs(y0)>1):  #
                v1 = 1           #
            v2 = (-1)*(abs(x0)/2)
        elif (abs(x0) == abs(y0)):
            v1 = abs(x0) 
            v2 = (-1)*(abs(x0)/2)
        elif (abs(x0)<abs(y0)):
            v1 = abs(y0)
            v2 = (-1)*(abs(y0)-abs(x0))
        else:
            v1 = 0
            v2 = 0
    
    #chay lui trai
    if (x0<-0.25) and (y0<-0.25):
        if (abs(x0)<abs(y0)):
            v1 = abs(y0)-abs(x0)
            v2 = (-1)*abs(y0)
        elif (abs(x0)==abs(y0)):
            v1 = abs(x0)/2
            v2 = (-1)*abs(x0)
        elif (abs(x0)>abs(y0)):
            v1 = abs(x0)/2
            v2 = (-1)*(abs(x0)+abs(y0))
            if (abs(x0) + abs(y0))>1 :  #
                v2 = (-1)*1           #
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

    #dung
    if (x0 == 0) and (y0 == 0):
        v1 = 0
        v2 = 0
       
       
    global v11
    global v22
    #xu li van toc cho dong co 1
    if v1 > 0  and v1 <= 1:
        v11=((v1*25)/1)*240
    elif v1 < 0  and v1 >= -1:
        v11=((v1*25)/1)*240
    else:
        v11= 0
    
    
    #xu li van toc cho dong co 2
    if v2 > 0  and v2 <= 1:
        v22=((v2*25)/1)*240
    elif v2 < 0 and v2 >= -1:
        v22=((v2*25)/1)*240
    else:
        v22= 0
       
    v11 = str(v11)
    v22 = str(v22)

    giatrigui = v11 + "," + v22  
    print("gia tri gui la:",giatrigui)
    pub.publish(giatrigui)  


        
def talker ():
    
    pub = rospy.Publisher('manual', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    rospy.loginfo("Publisher Node Started, now publishing messages")
    rospy.Subscriber('mytopic', String,callback)
    rospy.spin()


         	
if __name__== '__main__':
    try:
        talker()        
    except rospy.ROSInterruptException:
    	pass
