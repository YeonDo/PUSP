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
    rospy.init_node('test', anonymous=True)
    r = rospy.Rate(10)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 15)       ### TODO ###
    pub_land = rospy.Publisher('/ardrone/land', Empty)
    sub_ardrone= rospy.Subscriber('/ardrone/navdata', Navdata, callback)
    cmd=Twist()

    rospy.wait_for_service('/ardrone/setcamchannel')
    togglecam = rospy.ServiceProxy('/ardrone/setcamchannel',CamSelect)
    togglecam(0)
    t = int(np.pi*10)
    cmd.linear.x = 1
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.z = 0
    for i in range(30):
        pub_cmd_vel.publish(cmd)
        r.sleep()
    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.z = 0
    for i in range(20):
        pub_cmd_vel.publish(cmd)
        r.sleep()

    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.z = 1
    for i in range(t):
        pub_cmd_vel.publish(cmd)
        r.sleep()




if __name__=='__main__':
    # try:
    #         uav = PUSPLanding()
    #         count - 0
    main()


# except rospy.ROSInterruptException:
    # pass