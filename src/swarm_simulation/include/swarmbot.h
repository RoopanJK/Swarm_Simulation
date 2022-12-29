#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "goalconst.h"
#include <ros/rate.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "swarmbot_msgs/SwarmBotAction.h"
// #include "swarmbot_msgs/SwarmBotInte"

// #include<algorithm>
// #include<cmath>
// #include<strings.h>
// #include<tf/tf.h>
// #include<unistd.h>

class VelocityController
{
private:
    
    ros::NodeHandle nh_;

    Goal goal;
    std::string action_name;
    geometry_msgs::Twist cmd_msg;
    geometry_msgs::Twist *posePtr;
    geometry_msgs::Twist stop;
    swarmbot_msgs::SwarmBotResult result_;
    swarmbot_msgs::SwarmBotFeedback feedback_;
    boost::mutex BotInteruptMutex;

    actionlib::SimpleActionServer<swarmbot_msgs::SwarmBotAction> as_;
    ros::Publisher pub_cmdVel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::ServiceServer service = nh_.advertiseService("bot_stop", &VelocityController::servCallback, this);


public:

    VelocityController(geometry_msgs::Twist *_posePtr, std::string name)
     : as_(nh_, name, boost::bind(&VelocityController::executeCB, this, _1), false), action_name(name)
     {
        posePtr = _posePtr;
        stop.linear.x = 0;
        stop.linear.y = 0;
        stop.angular.z = 0;
        as_.start();
     }
     ~VelocityController()
     {
        as_.shutdown();
        pub_cmdVel.shutdown();
     }

     bool servCallback(swarmbot_msgs::s)


}