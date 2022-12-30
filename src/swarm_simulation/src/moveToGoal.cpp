#include "swarm_simulation/swarmbot.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/init.h>
#include <std_msgs/String.h>
#include <turtlesim/Pose.h>
#include <boost/core/ref.hpp>
#include <string>
#include <std_msgs/String.h>



turtlesim::Pose posemsg;
std::string bot_no;
std::string bot_name;

void pose_callback(const turtlesim::Pose &_pose)
{
    posemsg = _pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Goal Action Server");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    bot_no = (std::string)argv[1];
    bot_name = "swarmbot" + bot_no;
    ros::Subscriber pose = nh_.subscribe(bot_name + "/pose", 1000, pose_callback); 
    VelocityController controller(&posemsg, bot_name);
    ros::Rate loop_rate(20);
    ros::waitForShutdown();
    return 0;
}
