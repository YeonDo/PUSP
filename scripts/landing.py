#!/usr/bin/env python

# license removed for brevity

import rospy
import rospkg
import random
import numpy 		 as np
import sys
#import scipy.linalg 	 as la

from math 		 import *
from std_msgs.msg 	 import String
from std_msgs.msg 	 import Empty
from sensor_msgs.msg 	 import Joy
from geometry_msgs.msg   import Twist
from turtlesim.msg 	 import Pose
from ardrone_autonomy.msg import Navdata
# from std_srvs.srv       import Empty as Empty_1
from ardrone_autonomy.srv import CamSelect
### TODO: Change Param, param_msg, EKF ###



ll=0
tt=0
u=0
v=0
w=0
rr=0
rho=-999
px=-999
py=-999
tagcount=0
auto=0
eta=-999
rho_d=1
#land.last_status=0;



def callback(data): # read tags
    global tagcount, px, py, rho, eta 
    tagcount=data.tags_count
    rho = data.altd/1000  #convert mili to meters
    if tagcount>0:
        px_tmp = float(data.tags_xc[0])/1000.0
        py_tmp = float(data.tags_yc[0])/1000.0
        px=(px_tmp*640.0)-640.0/2.0
        py=(py_tmp*360.0)-360.0/2.0
        # rho=data2.tags_distance[0]/100 # convert into meters
        # fl=320/(np.tan(np.pi/6))
        # eta=np.arctan2(-px,fl)
        # print 'py=', 



# The Main Function
def main():
    rospy.init_node('landing', anonymous=True)
    r = rospy.Rate(10)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 15)       ### TODO ###
    # pub_takeoff = rospy.Publisher('/ardrone/takeoff',Empty)
    pub_land = rospy.Publisher('/ardrone/land', Empty)
    # sub = rospy.Subscriber('/joy', Joy,  callback1)
    sub_ardrone= rospy.Subscriber('/ardrone/navdata', Navdata, callback)
    cmd=Twist()
# A=buttons[0]
# B=buttons[1]
# X=buttons[2]
# Y=buttons[3]
# start =buttons[7]
# back =buttons[6]


# linear.x=axes[4]
# linear.y=axes[3]
# linear.z=axes[1]
# angular.z=axes[0]
    rospy.wait_for_service('/ardrone/setcamchannel')
    togglecam = rospy.ServiceProxy('/ardrone/setcamchannel',CamSelect)
    togglecam(1)
    count= 1
    step= 0
    hover = 0
    cc=0
    t = int(np.pi*10)
    print 'Bottom camera on'
    while not rospy.is_shutdown():
        d = np.sqrt(px*px + py*py)
        # print 'distance to tag =', d
        if (tagcount>0): 
            step = 0
            count =1
            cc = 0
            if (d>50): #tag detect and move to tag
                cmd.linear.x= -0.5*py
                cmd.linear.y= -0.5*px
                cmd.linear.z= -0.1
                cmd.angular.z= 0
                hover =0
            else:
                if (hover<10):
                    cmd.linear.x=0
                    cmd.linear.y=0
                    cmd.linear.z= 0
                    cmd.angular.z=0
                    hover+=1
                    print hover
                else:
                    print 'landing complete'
                    pub_land.publish(Empty())
                    sys.exit()
            pub_cmd_vel.publish(cmd)
            r.sleep()
        else:
            print step, count, cc
            if (step < count):
                cmd.linear.x= 1
                cmd.linear.y= 0
                cmd.linear.z= (rho_d-rho)/5
                cmd.angular.z= 0
                for i in range(30):
                    pub_cmd_vel.publish(cmd)
                    r.sleep()
                cmd.linear.x= 0
                cmd.linear.y= 0
                cmd.linear.z= 0
                cmd.angular.z= 0
                for i in range(10):
                    pub_cmd_vel.publish(cmd)
                    r.sleep()
                step+=1
            else:
                cmd.linear.x= 0
                cmd.linear.y= 0
                cmd.linear.z= (rho_d-rho)/5
                cmd.angular.z= 1
                for i in range(t):
                    pub_cmd_vel.publish(cmd)
                    r.sleep()
                cmd.linear.x= 0
                cmd.linear.y= 0
                cmd.linear.z= 0
                cmd.angular.z= 0
                for i in range(10):
                    pub_cmd_vel.publish(cmd)
                    r.sleep()
                step=0
                cc+=1
                if (cc>1):
                    count+=1
                    cc = 0
            
    rospy.loginfo("Sensors Node Has Shutdown...")
    rospy.signal_shutdown(0)





if __name__=='__main__':
    # try:
    #         uav = PUSPLanding()
    #         count - 0
    main()


# except rospy.ROSInterruptException:
    # pass