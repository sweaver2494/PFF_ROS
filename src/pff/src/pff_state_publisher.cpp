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
  double axle_length;
  double period; // seconds

  int right_wheel_angle;// radians
  int left_wheel_angle; // radians

  int x_position;
  int y_position;
  int orientation; // radians

  Robot(double w, double l, double p) :
    wheel_radius(w),
    axle_length(l),
    period(p)
  {}

} state(1.0, 0.8, 1.0);

std::unique_ptr<tf::TransformBroadcaster> broadcaster;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO_STREAM("Received Joint State");
  
  // Calculate angular velocity of wheels
  double new_right_wheel_angle = msg->position[0] / 8; // divide by 8 is cheating to avoid angle issues
  double new_left_wheel_angle = msg->position[1] / 8; // divide by 8 is cheating to avoid angle issues
  double u_right_wheel = (new_right_wheel_angle - state.right_wheel_angle) / state.period;
  double u_left_wheel = (new_left_wheel_angle - state.left_wheel_angle) / state.period;

  // Calculate linear velocity of robot
  double tempMult = (state.wheel_radius / 2) * (u_right_wheel + u_left_wheel);
  double v_x = tempMult * cos(state.orientation);
  double v_y = tempMult * sin(state.orientation);

  // Calculate change in robot orientation
  double v_angle = (state.wheel_radius / state.axle_length) * (u_right_wheel - u_left_wheel);

  // Update state
  state.right_wheel_angle = new_right_wheel_angle;
  state.left_wheel_angle = new_left_wheel_angle;
  state.x_position += v_x;
  state.y_position += v_y;
  state.orientation += v_angle;

  // Update and Send TF Transform
  geometry_msgs::TransformStamped tf_trans;

  tf_trans.header.frame_id = "map";
  tf_trans.child_frame_id = "center_axle";

  tf_trans.header.stamp = ros::Time::now();
  tf_trans.transform.translation.x = v_x;
  tf_trans.transform.translation.y = v_y;
  tf_trans.transform.translation.z = 0;
  tf_trans.transform.rotation = tf::createQuaternionMsgFromYaw(v_angle);

  broadcaster->sendTransform(tf_trans);
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
