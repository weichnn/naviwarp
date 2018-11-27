#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <string.h>
#include "stdio.h"
#include "opencv2/calib3d/calib3d.hpp"
// #include "opencv2/nonfree/features2d.hpp"
#include "opencv2/opencv_modules.hpp"

#include <opencv2/opencv.hpp>


#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"

#include "naviwarp/CommandBool.h"

using namespace cv;
using namespace std;

int state_mode;
geometry_msgs::PoseStamped pose;
int updated;

void GrabImage(const geometry_msgs::PoseStamped& msg)
{
    if (state_mode != 1)
    {
        return;
    }

    updated = true;
    pose = msg;
}

void GrabVICON(const geometry_msgs::PoseStamped& msg)
{
    if (state_mode != 0)
    {
        return;
    }
    updated = true;
    pose = msg;
}

bool switch_cb(naviwarp::CommandBool::Request  &req,
         naviwarp::CommandBool::Response &res)
{
    if(req.value == false)
    {
        state_mode = 0;
        ROS_INFO("switching to the SLAM");
    }
    if(req.value == true)
    {
        state_mode = 1;
        ROS_INFO("switching to the VICON");
    }
  return true;
}

 
int main(int argc, char** argv)
{


    std::cout << "start!" << std::endl;

    if (argc != 3) {
        cerr << endl << "not enough para input, the_exe vicon_object_name SLAM_topic" << endl;
        return 1;
    }

    state_mode = 0;
    updated = 0;

    ros::init(argc, argv, "Switcher");
    ros::NodeHandle nodeHandler;



    std::string viconTopic("/vicon/"); 
    viconTopic += argv[1];
    ros::Subscriber vicon_sub = nodeHandler.subscribe(viconTopic, 1, GrabVICON);

    
    ros::Subscriber sub = nodeHandler.subscribe(argv[2], 1, GrabImage);


    ros::ServiceServer service = nodeHandler.advertiseService("doSwitch", switch_cb);
    ROS_INFO("Ready to switch.");

    ros::Publisher chatter_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);

    while(ros::ok())
    {

        if (updated == true)
        {
            chatter_pub.publish(pose);
            updated = false;
            ros::spinOnce();
        }


    }

    return 0;
}
