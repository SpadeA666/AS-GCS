#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_simple_goal");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal1;

  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();

  goal1.target_pose.pose.position.x = 3.0;
  goal1.target_pose.pose.position.y = -2.0;
  goal1.target_pose.pose.orientation.z = 1.0;

  move_base_msgs::MoveBaseGoal goal2;

  goal2.target_pose.header.frame_id = "map";

  goal2.target_pose.pose.position.x = 0.0;
  goal2.target_pose.pose.position.y = 0.0;
  goal2.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal1");
  ac.sendGoal(goal1);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("goal1 complete!");

    goal2.target_pose.header.stamp = ros::Time::now();

    ros::Duration(1.0).sleep();

    ac.sendGoal(goal2);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Goal 2 reached! All missions complete!");
    }
    else
    {
      ROS_INFO("Goal 2 failed ...");
    }
  }
  else
  {
    ROS_INFO("Mission failed ...");
  }
  return 0;
}