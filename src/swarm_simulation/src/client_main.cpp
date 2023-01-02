#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <swarmbot_msgs/SwarmBotAction.h>
#include <swarmbot_msgs/SwarmBotGoal.h>
#include <swarmbot_msgs/SwarmBotResult.h>
#include <ros/rate.h>
#include <string.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <turtlesim/Pose.h>
#include <stdint.h>
#include "swarm_simulation/goalconst.h"

void callback(std_msgs::Int64 data);
void servodir();
void clientRoutine();
void pose_callback(const turtlesim::Pose &_pose);

actionlib::SimpleActionClient<swarmbot_msgs::SwarmBotAction> client("swarmbot", true);
geometry_msgs::Twist cmd_vel;
geometry_msgs::Twist stop;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener listener(tfBuffer);
swarmbot_msgs::SwarmBotResultConstPtr result;
swarmbot_msgs::SwarmBotGoal goal;
std_msgs::Int32 col_res;
std_msgs::Int32 ser_res;

std::string bot_no;
std::string bot_name;
turtlesim::Pose pose_msg;
std::string node_name;

ros::Publisher pub_servo;
ros::Publisher pub_colorreq;
ros::Publisher pub_cmdVel;

bool callbackCalled = false;

int main(int argc, char **argv)
{
    bot_no = (std::string)argv[1];
    ros::init(argc, argv, "Action_client for swarmbot" + bot_no);
    ros::NodeHandle nh_;
    pub_servo = nh_.advertise<std_msgs::Int32>("servo", 1000);
    pub_colorreq = nh_.advertise<std_msgs::Int32>("color_req", 1000);
    pub_cmdVel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    bot_name = "swarmbot" + bot_no;
    stop.linear.x = 0;
    stop.linear.y = 0;
    stop.linear.z = 0;
    stop.angular.x = 0;
    stop.angular.y = 0;
    stop.angular.z = 0;
    ros::Subscriber sub = nh_.subscribe<std_msgs::Int64>("dest", 1, callback);
    ros::Subscriber pose_action = nh_.subscribe(bot_name + "/pose", 10, pose_callback);
    client.waitForServer();
    callbackCalled = false;
    clientRoutine();
    ros::waitForShutdown();
    return 0;
}

void callback(std_msgs::Int64 data)
{
    callbackCalled = true;
    ROS_INFO("Sending Goal");
    goal.index = data.data;
    client.sendGoal(goal);
    client.waitForResult();
    result = client.getResult();
    ROS_INFO("Result Received");
    ros::Duration(0.5).sleep();
    servodir();
    ros::Duration(1.5).sleep();
    // cmd_vel.linear.x = servopush;
    // pub_cmdVel.publish(cmd_vel);
    // ros::Duration(1.0).sleep();
    pub_cmdVel.publish(stop);
    ros::param::set("package_id", 0);
    ros::param::set("package_string", "Home");
    ros::param::set("package_id", 0);
    goal.index = -1 * data.data;
    client.sendGoal(goal);
    client.waitForResult();
    result = client.getResult();
    callbackCalled = false;
    ros::Duration(1.0).sleep();
}

void clientRoutine()
{
    while (ros::ok())
    {
        int control_bit;
        ros::param::get("bot_control", control_bit);
        if (control_bit == 1)
        {
            while (!callbackCalled)
            {
                ROS_INFO("Waiting for Color data");
                if (result->inductIndex != 0)
                {
                    col_res.data = result->inductIndex;
                    pub_colorreq.publish(col_res);
                    callbackCalled = true;
                    ros::Duration(3.0).sleep();
                }
                else
                {
                    std::cout << "Initiating" << std::endl;
                    // Find Current Induct and publish
                    if (abs(pose_msg.y - yInduct[0]) < 0.2)
                    {
                        col_res.data = 1;
                        pub_colorreq.publish(col_res);
                        callbackCalled = true;
                    }
                    else if (abs(pose_msg.y - yInduct[1]) < 0.2)
                    {
                        col_res.data = 2;
                        pub_colorreq.publish(col_res);
                        callbackCalled = true;
                    }
                    else
                    {
                        std::cout << "Not in Induct" << std::endl;
                        goal.index = 0;
                        client.sendGoal(goal);
                        client.waitForResult();
                        result = client.getResult();
                        ROS_INFO("Got result");
                        ros::Duration(0.5).sleep();
                    }
                }
            }
        }
    }
}

void servodir()
{
    int dest = result->destIndex;
    if (dest == 1 || dest == 4 || dest == 7)
    {
        ser_res.data = -1;
        pub_servo.publish(ser_res);
        ROS_INFO("Actuating servo in -1");
    }
    else
    {
        ser_res.data = 1;
        pub_servo.publish(ser_res);
        ROS_INFO("Actuating servo in 1");
    }
}

void pose_callback(const turtlesim::Pose &_pose)
{
    pose_msg = _pose;
}