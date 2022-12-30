#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "goalconst.h"
#include <ros/rate.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlesim/Pose.h>
#include "swarmbot_msgs/SwarmBotAction.h"
#include "swarmbot_msgs/SwarmBotInterrupt.h"
#include "swarmbot_msgs/SwarmBotInterruptRequest.h"
#include "swarmbot_msgs/SwarmBotInterruptResponse.h"

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
   turtlesim::Pose *posePtr;
   geometry_msgs::Twist stop;
   swarmbot_msgs::SwarmBotResult result_;
   swarmbot_msgs::SwarmBotFeedback feedback_;
   boost::mutex SwarmBotInteruptMutex;

   int lastDest = 0;

   actionlib::SimpleActionServer<swarmbot_msgs::SwarmBotAction> as_;
   ros::Publisher pub_cmdVel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
   ros::ServiceServer service = nh_.advertiseService("bot_stop", &VelocityController::servCallback, this);

public:
   VelocityController(turtlesim::Pose *_posePtr, std::string name)
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

   bool servCallback(swarmbot_msgs::SwarmBotInterruptRequest &req,
                     swarmbot_msgs::SwarmBotInterruptResponse &res)
   {
      if (req.pause == 1)
      {
         ros::param::set("swarmbot_state", 0);
         pub_cmdVel.publish(stop);
         feedback_.xVel = 0;
         feedback_.yVel = 0;
         as_.publishFeedback(feedback_);
         SwarmBotInteruptMutex.lock();
         ROS_INFO("Stopping the robot");
      }
      if (req.pause == 2)
      {
         pub_cmdVel.publish(stop);
         SwarmBotInteruptMutex.lock();
         pub_cmdVel.publish(stop);
         ROS_INFO("Stopping the robot");
      }
      if (req.pause == 0)
      {
         SwarmBotInteruptMutex.unlock();
         ROS_INFO("Resuming the robot");
         ros::Duration(0.2).sleep();
         ros::param::set("swarmbot_state", 1);
      }
      return true;
   }

   void executeCB(const swarmbot_msgs::SwarmBotGoalConstPtr &goal)
   {
      ros::Rate loop_rate(20);
      std::string goalID;
      int induct;
      ROS_INFO("Received the Goal %i", goal->index);

      if (goal->index > 0)
      {
         induct = findInduct();
         goalID = std::to_string(induct) + "_" + std::to_string(abs(goal->index));
      }

      if (goal->index < 0)
      {
         induct = findNearInduct();
         goalID = "r_" + std::to_string(induct) + "_" + std::to_string(abs(goal->index));
      }

      if (goal->index == 0)
      {
         induct = findNearInduct();
         goalID = "r_" + std::to_string(induct) = "_0";
      }

      feedback_.goalPoint = goal->index;
      feedback_.inductPoint = induct;
      ROS_INFO("Executing plan from %s", goalID.c_str());
      auto hashmapFound = umap.find(goalID);
      std::vector<Goal> waypoint = hashmapFound->second;
      int i = 0;
      for (Goal goalPoint : waypoint)
      {
         if (as_.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name.c_str());
            as_.setPreempted();
            break;
         }
         setGoal(goalPoint);
         if (i == 1)
         {
            ros::param::set("swarbot_state", 1);
            if (goal->index > 0)
               ros::param::set("/induct" + std::to_string(induct) + "_occupancy", 0);
            ROS_INFO("Induct Freed");
         }

         // To check Induct Occupancy in Induct zones
         if (goalPoint.checkPoint == 1)
         {
            int value = 0;
            ROS_INFO("Checkpoint reached: Checking for Occupancy in %d", induct);
            while(ros::param::get("/induct" + std::to_string(induct) + "_occupancy", value));
            {
               ros::param::set("/induct" + std::to_string(induct) + "_wait", 1);
               if (value == 0)
               {
                  ros::param::set("/induct" + std::to_string(induct) + "_wait", 0);
                  ROS_INFO("Induct %d Freed", induct);
                  break;
               }
               ros::Rate(0.5).sleep();
            }
            ROS_INFO("Occupying Induct %d", induct);
            ros::param::set("/induct" + std::to_string(induct) + "_occupancy", 1);
         }

         ROS_INFO("Moving in %c to point %i", axisToString(goalPoint.axis), goalPoint.point);
         while (!inTolerance())
         {
            cmd_msg = calculateVelocity();
            feedback_.axis = axisToString(goalPoint.axis);
            feedback_.point = goalPoint.point;
            feedback_.Xpoint = posePtr->x;
            feedback_.Ypoint = posePtr->y;
            feedback_.xVel = cmd_msg.linear.x;
            feedback_.yVel = cmd_msg.linear.y;
            as_.publishFeedback(feedback_);
            SwarmBotInteruptMutex.lock();
            pub_cmdVel.publish(cmd_msg);
            SwarmBotInteruptMutex.unlock();
            if (inTolerance())
            {
               pub_cmdVel.publish(stop);
               break;
            }
            loop_rate.sleep();
         }
         lastDest = goal->index;
         pub_cmdVel.publish(stop);
         ROS_INFO("X - %lf Y- %lf", posePtr->x, posePtr->y);
         i++;
      }
      result_.destIndex = goal->index;
      result_.inductIndex = induct;
      ros::param::set("swarmbot_state", 0);
      as_.setSucceeded(result_);
   }

   /**
    * @brief Sets the Goal received to the class goal
    *
    * @param _goal Required Goal position
    *
    * @returns void
    *
    */
   void setGoal(Goal _goal)
   {
      this->goal = _goal;
   }

   /**
    * @brief Convert incoming Axis Datatype to string data type
    *
    * @param _axis Input Axis datatype
    *
    * @returns Corresponding axis relevant to input _axis
    *
    */
   char axisToString(Axis _axis)
   {
      if (_axis == cx)
         return 'x';
      else
         return 'y';
   }

   /** @brief Calculate distance to goal point
    *
    * @returns Remaining Distance left to goal
    *
    */
   double euclidianDistance()
   {
      double _distance = 0.0;
      if (goal.axis == cx)
         _distance = cxPoint[goal.point - 1] - posePtr->x;

      if (goal.axis == cy)
         _distance = cyPoint[goal.point - 1] - posePtr->y;

      return _distance;
   }

   /**
    * @brief Checks if the swarmbot has reached its goal or not
    *
    *
    * @returns true or false
    */
   bool inTolerance()
   {
      if (abs(euclidianDistance()) < 0.05)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * @brief Calculate linear and angular Velocity
    *
    * @returns cmd_vel as Twist
    */
   geometry_msgs::Twist calculateVelocity()
   {
      geometry_msgs::Twist _twist;
      if (goal.axis == cx)
      {
         double _linearVel = euclidianDistance();
         _twist.linear.x = _linearVel;
      }

      if (goal.axis == cy)
      {
         double _linearVel = euclidianDistance();
         _twist.linear.y = _linearVel;
      }

      if (abs(posePtr->theta) > 0.05)
      {
         double _angularVel = 1;
         _twist.angular.z = _angularVel;
      }
      else
      {
         double _angularVel = -1;
         _twist.angular.z = _angularVel;
      }
      return _twist;
   }

   /**
    * @brief Used to Calculate the Nearest Induct zone
    *
    * @returns int flag used to specify induct
    */
   int findInduct()
   {
      if (abs(posePtr->y - yInduct[0]) < 0.01)
      {
         return 1;
      }
      else if (abs(posePtr->y - yInduct[1]) < 0.01)
      {
         return 2;
      }
      return 0;
   }

   /**
    * @brief Used to find the closest partitioned Induct from the center
    * 
    * 
    * @returns Closest induct from current position
    * 
    */
   int findNearInduct()
   {
      if (posePtr->y < cyPoint[4])
      {
         return 1;
      }
      else
      {
         return 2;
      }
   }
};
