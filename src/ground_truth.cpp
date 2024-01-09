/**
 * @file
 * Convert ground truth pose (optitrack frame) to nav_msgs/Path msg in robot local frame.
 * @date 2024-01-09
 * @version 1.0.0
 * @Copyright 2024 <siyuanwu99@gmail.com>
 */

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>

ros::Publisher path_pub_;
ros::Publisher pose_pub_;

Eigen::Isometry3d tf_world2local_ = Eigen::Isometry3d::Identity();
nav_msgs::Path    path_msg_;
double            t0_;

void boardCallback(const geometry_msgs::PoseStamped &msg) {
  Eigen::Vector3d board_pos_world(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  std::cout << "pos:" << board_pos_world.transpose() << std::endl;

  /* transfrom to body frame */
  Eigen::Vector3d board_pos_body;

  board_pos_body = tf_world2local_.matrix().block<3, 3>(0, 0) * board_pos_world +
                   tf_world2local_.matrix().block<3, 1>(0, 3);

  ROS_INFO("board pos in body frame: %f, %f, %f", board_pos_body(0), board_pos_body(1),
           board_pos_body(2));

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp       = ros::Time::now();
  pose_msg.header.frame_id    = "map";
  pose_msg.pose.position.x    = board_pos_body(0);
  pose_msg.pose.position.y    = board_pos_body(1);
  pose_msg.pose.position.z    = board_pos_body(2);
  pose_msg.pose.orientation.w = 1;
  pose_msg.pose.orientation.x = 0;
  pose_msg.pose.orientation.y = 0;
  pose_msg.pose.orientation.z = 0;

  pose_pub_.publish(pose_msg);

  path_msg_.header.stamp    = ros::Time::now();
  path_msg_.header.frame_id = "map";
  path_msg_.poses.push_back(pose_msg);
  path_pub_.publish(path_msg_);
}

void bodyCallback(const geometry_msgs::PoseStamped &msg) {
  Eigen::Vector3d    vec(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  Eigen::Quaterniond rot(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                         msg.pose.orientation.z);
  // std::cout << "body pos: " << vec.transpose() << std::endl;
  Eigen::Isometry3d tf_body2world;
  tf_body2world.matrix().block<3, 3>(0, 0) = rot.toRotationMatrix();
  tf_body2world.matrix().block<3, 1>(0, 3) = vec;

  tf_world2local_ = tf_body2world.inverse();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tag2path");
  ros::NodeHandle nh("~");

  /** subscribe odometry msgs from optitrack */
  ros::Subscriber odom_sub = nh.subscribe("caliboard_pose", 1, boardCallback);
  ros::Subscriber body_sub = nh.subscribe("body_pose", 1, bodyCallback);

  /** publish tag pose msg */
  path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("local_pose", 1);

  /* initialize */
  path_msg_.header.stamp    = ros::Time::now();
  path_msg_.header.frame_id = "map";
  t0_                       = ros::Time::now().toSec();

  ros::spin();
  return 0;
}
