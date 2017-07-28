#include <iostream>
#include <math.h>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>

#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

struct Robot
{
  double wheel_radius;
  double angle_length;

  double time;	// seconds

  int right_wheel_angle;
  int left_wheel_angle;
  int orientation;

  Robot(double w, double l) :
    wheel_radius(w),
    angle_length(l)
  {}

} state(1.0, 0.8);

std::unique_ptr<tf::TransformBroadcaster> broadcaster;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO_STREAM("Received Joint State");

  // Calculate period
  ros::Time new_time = ros::Time::now();
  double period = new_time.toSec() - state.time;
  
  // Calculate angular velocity of wheels
  double u_right_wheel = (msg->position[0] - state.right_wheel_angle) / period;
  double u_left_wheel = (msg->position[1] - state.left_wheel_angle) / period;

  // Calculate linear velocity of wheels
  double tempMult = (state.wheel_radius / 2) * (u_right_wheel + u_left_wheel);
  double v_x = tempMult * sin(state.orientation);
  double v_y = tempMult * cos(state.orientation);

  // Calculate change in robot orientation
  double v_angle = (state.wheel_radius / state.angle_length) * (u_right_wheel - u_left_wheel);
  double new_angle = state.orientation + v_angle;

  ROS_INFO_STREAM("Sending TF Transform");

  // Update and Send TF Transform
  geometry_msgs::TransformStamped tf_trans;

  tf_trans.header.frame_id = "map";
  tf_trans.child_frame_id = "center_axle";

  tf_trans.header.stamp = new_time;
  tf_trans.transform.translation.x = v_x;
  tf_trans.transform.translation.y = v_y;
  tf_trans.transform.translation.z = 0;
  tf_trans.transform.rotation = tf::createQuaternionMsgFromYaw(new_angle);

  broadcaster->sendTransform(tf_trans);


  // Update state
  state.time = new_time.toSec();
  state.right_wheel_angle = msg->position[0];
  state.left_wheel_angle = msg->position[1];
  state.orientation = new_angle;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pff_state_publisher");

  ROS_INFO_STREAM("Initialized PFF State Publisher node.");

  ros::NodeHandle n;

  broadcaster.reset(new tf::TransformBroadcaster());

  ros::Subscriber joint_sub = n.subscribe("joint_states", 1, jointStateCallback);

  ros::spin();

  return 0;
}
