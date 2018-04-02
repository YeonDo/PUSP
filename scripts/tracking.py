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
from ardrone_autonomy.srv       import CamSelect
### TODO: Change Param, param_msg, EKF ###



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
td=0
#land.last_status=0;



def callback(data): # read tags
    global tagcount, px, py, rho, eta 
    tagcount=data.tags_count
    rho = data.altd/1000
    if tagcount>0:
        px_tmp = float(data.tags_xc[0])/1000.0
        py_tmp = float(data.tags_yc[0])/1000.0
        px=(px_tmp*640.0)-640.0/2.0
        py=(py_tmp*360.0)-360.0/2.0
        td=data.tags_distance[0]/100 # convert centi into meters
        print 'distance=',td , 'm'
        # fl=320/(np.tan(np.pi/6))
        # eta=np.arctan2(-px,fl)
        # print 'eta=', eta*(180/np.pi)
        # print 'py=', py
    						
### TODO ###



# The Main Function
def main():
    rospy.init_node('tracking', anonymous=True)
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
    phase = 1
    hover = 0
    while not rospy.is_shutdown():
        d = np.sqrt(px*px + py*py)
        # print 'distance to tag =', d
        # if (auto==1): #drone search tag
        #     # count+=1
        if (phase==1):
            if (tagcount>0): 
                if (td>0.5): #tag detect and move to tag
                    u_vision= 1
                    v_vision= -2*px/100
                    w_vision= -2*py/100
                    r_vision= 0
                    hover =0
                else: #ready to land
                    if (hover<5):
                        u_vision=0
                        v_vision=0
                        w_vision=0
                        r_vision=0
                        hover+=1
                    else:
                        rospy.wait_for_service('/ardrone/setcamchannel')
                        togglecam = rospy.ServiceProxy('/ardrone/setcamchannel',CamSelect)
                        togglecam(1)
                        phase+=1
                        print 'toggle cam'
            else:
                u_vision= 0
                v_vision= 0
                w_vision= (rho_d-rho)*5
                r_vision= 1
        if (phase==2):
            if (tagcount>0):
                if (d>50):
                    u_vision= -2*py/100
                    v_vision= -2*px/100
                    w_vision= -0.1
                    r_vision= 0
                else:
                    pub_land.publish(Empty())
                    sys.exit()
            else:
                u_vision= 0
                v_vision= 0
                w_vision= 0.5
                r_vision= 0
        cmd.linear.x= u_vision
        cmd.linear.y= v_vision
        cmd.linear.z= w_vision
        cmd.angular.z= r_vision
        pub_cmd_vel.publish(cmd)
        # if tt ==1:
        #     pub_takeoff.publish(Empty())
        # if ll ==1:
        #     pub_land.publish(Empty())
        r.sleep()
    rospy.loginfo("Sensors Node Has Shutdown...")
    rospy.signal_shutdown(0)




      



if __name__=='__main__':
    # try:
    #         uav = PUSPLanding()
    #         count - 0
    main()


# except rospy.ROSInterruptException:
    # pass