#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <turtlesim/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

double roll, pitch, yaw;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tf2Pose");
    ros::NodeHandle n;
    
    std::cout<<"Initiating lookup for bot: "<< argv[1];
    
    std::string marker_id = "marker_id";
    std::string bot_id = marker_id.append(argv[1]);

    std::string bot_publihser = "swarmbot";
    bot_publihser.append(argv[1]);
    bot_publihser.append("/pose");

    // std::string bot_pubisher_ = "swarmbot" + std::to_string(abs(argv[1]));

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tflistener(tfBuffer);

    ros::Publisher swarmbot_pose = n.advertise<turtlesim::Pose>(bot_publihser, 1);

    ros::Rate rate(10.0);

    while(n.ok())
    {
        geometry_msgs::TransformStamped transform;
        try{
            transform = tfBuffer.lookupTransform("reference", bot_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        turtlesim::Pose msg;
        tf2::Quaternion quat;
        quat = {transform.transform.rotation.x, transform.transform.rotation.y,
                transform.transform.rotation.z, transform.transform.rotation.w};
        
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        msg.x = transform.transform.translation.x;
        msg.y = transform.transform.translation.y;
        msg.theta = yaw;
        // std::cout << "x= " << msg.x << std::endl;
        // std::cout << "y = "<< msg.y << std::endl;
        // std::cout << "z = "<< yaw << std::endl;
        
        swarmbot_pose.publish(msg);
    }
}


