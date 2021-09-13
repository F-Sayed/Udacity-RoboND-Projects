#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

float pickup_location[2] = {-2.5, -2.5};
float drop_location[2] = {3.25, 0.5};
float current_position[2] = {0.0, 0.0};

void get_robot_position(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_position[0] = msg->pose.pose.position.x;
  current_position[1] = msg->pose.pose.position.y;
}

float calculate_distance(float goal_pos[2])
{
  return sqrt(pow(current_position[0] - goal_pos[0], 2) + pow(current_position[1] - goal_pos[1], 2));
}

bool reached_goal(float goal_pos[2])
{
  return calculate_distance(goal_pos) < 1.0;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;


  enum state {pickup_zone, drop_zone, hide};
  state marker_state = pickup_zone;

  ros::Subscriber position_sub = n.subscribe("odom", 10, get_robot_position);
    

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    
    
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    switch(marker_state)
    {
      case pickup_zone:
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pickup_location[0];
        marker.pose.position.y = pickup_location[1];
        marker.pose.position.z = 0.5;
        marker_pub.publish(marker);
        ROS_INFO("Marker initialized at pickup zone");
        while(!reached_goal(pickup_location))
        {
	ros::spinOnce();
        }
        ROS_INFO("Robot reached pickup zone, simulating pickup");
        sleep(5);
        marker_state = drop_zone;
	
        break;
      case drop_zone:
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = drop_location[0];
        marker.pose.position.y = drop_location[1];
        marker.pose.position.z = 0.5;
        marker_pub.publish(marker);
        ROS_INFO("Marker initialized at drop zone");
        while(!reached_goal(drop_location))
        {
  ros::spinOnce();
        }
        ROS_INFO("Robot reached drop zone");
        marker_state = hide;
        break;
      case hide:
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        ROS_INFO("Marker hidden");
        sleep(5);
        break;
    }    
  }
}
