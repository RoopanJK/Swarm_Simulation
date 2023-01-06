#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <swarmbot_msgs/SwarmBotAction.h>
#include <swarmbot_msgs/SwarmBotGoal.h>
#include <swarmbot_msgs/SwarmBotResult.h>
#include <swarmbot_msgs/SwarmBotInterrupt.h>
#include <ros/rate.h>
#include <string.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <turtlesim/Pose.h>
#include <stdint.h>
#include "swarm_simulation/goalconst.h"

std::string bot_no;

class actionClient
{
private:
    ros::NodeHandle nh_;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist stop;
    swarmbot_msgs::SwarmBotResultConstPtr result;
    swarmbot_msgs::SwarmBotGoal goal;
    std_msgs::Int32 col_res;
    std_msgs::Int32 ser_res;
    tf2_ros::Buffer tfBuffer;
    std::string bot_name;
    turtlesim::Pose pose_msg;
    std::string node_name;
    ros::Subscriber pose_action;
    ros::Subscriber sub;
    actionlib::SimpleActionClient<swarmbot_msgs::SwarmBotAction> client;
    ros::Publisher pub_servo = nh_.advertise<std_msgs::Int32>("servo", 1000);
    ros::Publisher pub_colorreq = nh_.advertise<std_msgs::Int32>("color_req", 1000);
    ros::Publisher pub_cmdVel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    bool callbackCalled = false;

public:
    actionClient(std::string _bot_no) : client("swarmbot", true)
    {
        bot_name = "swarmbot" + _bot_no;
        tf2_ros::TransformListener listener(tfBuffer);
        sub = nh_.subscribe("/dest", 1, &actionClient::callback, this);
        pose_action = nh_.subscribe(bot_name + "/pose", 1, &actionClient::pose_callback, this);
        client.waitForServer();
        callbackCalled = false;
    }

    void callback(std_msgs::Int64 data)
    {
        std::cout << data.data << std::endl;
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
};

int main(int argc, char **argv)
{
    bot_no = std::string(argv[1]);
    ros::init(argc, argv, "Action_client_swarmbot" + bot_no);
    // ROS_INFO("Started client node for swarmbot_%s", &bot_no);
    actionClient ac(bot_no);
    ac.clientRoutine();
    ros::waitForShutdown();
    return 0;
}