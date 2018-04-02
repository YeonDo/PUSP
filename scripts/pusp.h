#pragma once
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
#ifndef __PUSP_H
#define __PUSP_H
 
 
 
 
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "cvd/thread.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"




class pusp : public QWidget
{
    void run();

    bool keepRunning;

private:
    ros::Publisher vel_pub;
    ros::Subscriber navdata_sub;
    ros::Publisher land_pub;
    ros::ServiceClient toggleCam_srvs;
    std_srvs::Empty toggleCam_srv_srvs;

    ros::NodeHandle nh;

    static pthread_mutex_t send_CS;

 public:

 	// callbacks
 	void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
 	void landCb(std_msgs::EmptyCOnstPtr);
 	void velCb(const geometry_msgs::TwistConstPtr vel);

 	void sendTogglecam;



};

#endif /* __TUMARDRONEGUI_H */

