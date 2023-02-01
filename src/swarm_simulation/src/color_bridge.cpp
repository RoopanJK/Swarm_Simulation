#include <ros/ros.h>
#include <ros/rate.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <swarmbot_msgs/SwarmBotInterrupt.h>
#include <swarmbot_msgs/PackageDetail.h>

ros::Publisher pub;
ros::Rate rate(10);
std::string bot_no;
ros::Subscriber sub;
swarmbot_msgs::PackageDetail response;

class color_bridge
{
public:
    color_bridge(std::string _bot_no, std::string method)
    {
        ros::NodeHandle nh_;
        pub = nh_.advertise<std_msgs::Int64>("dest", 100);
        bot_no = _bot_no;
        if (method == "2")
        {
            // Color detection from swarbot to identify packages
        }
        if (method == "1")
        {
            std::cout << "From file" << std::endl;
            ros::service::waitForService("/package_server");
            sub = nh_.subscribe("colorReq", 1, &color_bridge::callbackROS);
        }

    }

    void callbackROS()
    {
        try
        {
            std::cout << "Calling package server" << std::endl;
            response = ros::ServiceClient::call("/package_server", swarmbot_msgs::PackageDetail);
            ros::ser
        }
    }
}