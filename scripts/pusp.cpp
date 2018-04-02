 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#define _USE_MATH_DEFINES

#include <pusp.h>
#include "ros/ros.h"
#include "std_msgs"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include <cmath>


// constants (variances assumed to be present)
int dd = 0;
int d = 0;
int v = 0;
int w = 0;
int t = 0;
int rho = -999;
int px = -999;
int py = -999;
int tagcount = 0 ;
int eta = -999;
int angle = 0;

pthread_mutex_t pusp::send_CS = PTHREAD_MUTEX_INITIALIZER;

	
void pusp::callback1(const std_msgs::StringConstPtr& str)
{
	tagcount = str.tags_count
	if (tagcount >1)
	{
		px_tmp = float(str.tags_xc[0])/1000;
		py_tmp = float(str.tags_yc[0])/1000;
		px = (px_tmp*640.0)-320.0;
		py = (py_tmp*360.0)-180.0;
		rho = str.tags_distance[0]/100;
		angle = str.tags_orientation;
	}
}

//void pusp::callback2(const std_msgs::StringConstPtr& str)
//{
//	cs = str.camera_source;
//}

void pusp::sendToggleCam()
{
	pthread_mutex_lock(&send_CS);
	toggleCam_srv.call(togglecam_srv_srvs);
	pthread_mutex_unlock(&send_CS);
}

void pusp::run()
{

		std::cout << "Starting pusp vision tracking" << std::endl;

	vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"),1);
	land_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/land"),1);
	toggleCam_srv = nh.serviceClient<std_srvs::Empty>(nh.resolveName("ardrone/togglecam"),1);
	navdata_sub = nh.subscribe("ardrone/navdata", 1 , callback1);
	navdata_vd = nh.subscribe("ardrone/navdata_vision_detect", 1 ,callback2);



	while (keepRunning && nh.ok())   
	{
		d = px^2+py^2;
		if (tagcount>0) //tracking front camera
		{
			if (d>2500) && (rho < 0.5)
			{
				cmd.linear.x = kx*py;
				cmd.linear.y = -ky*px;
				cmd.linear.z = kz*(rho_d - rho);
				cmd.angular.z = 0;
			}
			else  //change to bottom camera
			{
				toggleCam_srv.sendToggleCam();
				dd = px^2+py^2;
				count = 1;
				if (tagcount>0)
					{
					if (dd>2500)
						{
							circum = 2 * math.pi * count/10;
							v = circum / 10;
							w = v/count*10;
							cmd.linear.x = v;
							cmd.linear.y = 0
							cmd.linear.z = 0;
							cmd.angular.z = w;
							count+=1
						}
					else
						{
							land_pub.publish();
						}	
					}
				else
					{
						cmd.linear.x = 0;
						cmd.linear.y = 0
						cmd.linear.z = -1;
						cmd.angular.z = 0;
					}
				pub_cmd_vel.publish(cmd)

			}	
		}
		else
		{
			cmd.linear.x = 0;
			cmd.linear.y = 0
			cmd.linear.z = 1;
			cmd.angular.z = 0;
		}
		pub_cmd_vel.publish(cmd)
	} 



	ROS_INFO("Sensors Node Has Shutdown...")
	ros::spin();	
	loop_rate.sleep();

	return 0;
}