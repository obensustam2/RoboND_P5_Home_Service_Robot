#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>
#include<iostream>
using namespace std;

// bool marker_reach_state = false;
uint8_t goal_reach_state = 0;

/* robot goal proximity callback function */
void goalReachCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  goal_reach_state = msg->data;
  // if (goal_reach_state==2)
  //   ROS_INFO("2 - Drop-Off goal is reached");
  // else if (goal_reach_state==1)
  //   ROS_INFO("1 - Pick-Up goal is reached");
  
  return;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/goal_reached", 1, goalReachCallback);

  while (ros::ok())
  {
    ros::spinOnce();

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = 1;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    switch(goal_reach_state)
    {
      case 0: // Publish the marker at the pick up zone 
      {
        marker.action = 0;
        marker.pose.position.x = -0.87;
        marker.pose.position.y = -0.52;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0.06;
        marker.pose.orientation.w = 0.99;
        ROS_INFO("Marker is published at Pick-up goal");
        break;
      }

      case 1: // Hide the marker 
      {
        marker.action = 2;
        //marker_pub.publish(marker);
        ROS_INFO("Marker is hided");
        break;
      }

      case 2: // Publish the marker at the drop off zone
      {
        marker.action = 0;
        marker.pose.position.x = 1.96;
        marker.pose.position.y = 0.79;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = -0.71;
        marker.pose.orientation.w = 0.70;
        marker_pub.publish(marker);
        ROS_INFO("Marker is published at Drop-Off goal");
        goal_reach_state=0;
        sleep(3);
        break;
      }
    } // switch

    marker_pub.publish(marker); // Publish the marker
      
    r.sleep();
  } // while
} // main