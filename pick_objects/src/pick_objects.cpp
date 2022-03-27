#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;
  ros::Publisher goal_reach_pub = n.advertise<std_msgs::UInt8>("/goal_reached", 1);
  std_msgs::UInt8 status_msg;  // goal reach status

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  while(1)
  {
    // First Goal
    goal.target_pose.pose.position.x = 0.01;
    goal.target_pose.pose.position.y = -1.98;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.70;
    goal.target_pose.pose.orientation.w = 0.71;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending Pick-Up goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Pick-Up goal is reached");
      status_msg.data = 1;
      goal_reach_pub.publish(status_msg);
    }
    else
      ROS_INFO("The base failed to reach the Pick-Up goal");
    
    sleep(5);
    
    // Second Goal
    goal.target_pose.pose.position.x = 2.01;
    goal.target_pose.pose.position.y = 0.56;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0.99;
    goal.target_pose.pose.orientation.w = 0.10;

    ROS_INFO("Sending Drop-Off goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Drop-Off goal is reached");
      status_msg.data = 2;
      goal_reach_pub.publish(status_msg);
    }
    else
      ROS_INFO("The base failed to reach the Drop-Off goal");
    
    sleep(5);
  
  }
  return 0;
}