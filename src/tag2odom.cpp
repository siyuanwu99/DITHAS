/**
 * @file
 * @brief
 *   This node will subscribe apriltag detection msg from apriltag node and convert to
 * geometry_msgs/PoseStamped msg in local frame.
 *
 * @date 27/12/2023
 * @version 1.0.0
 * @Copyright 2023 <siyuanwu99@gmail.com>
 */

#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <dithas/utils.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>

ros::Publisher pose_pub_;

Eigen::Affine3d tf_tag2cam_  = Eigen::Affine3d::Identity();
Eigen::Affine3d tf_cam2base_ = Eigen::Affine3d::Identity();

void tagCallback(const apriltag_ros::AprilTagDetectionArray &msg) {
  if (msg.detections.size() > 0) {
    Eigen::Quaterniond rot_tag2cam(msg.detections[0].pose.pose.pose.orientation.w,
                                   msg.detections[0].pose.pose.pose.orientation.x,
                                   msg.detections[0].pose.pose.pose.orientation.y,
                                   msg.detections[0].pose.pose.pose.orientation.z);
    Eigen::Vector3d    vec_tag2cam(msg.detections[0].pose.pose.pose.position.x,
                                   msg.detections[0].pose.pose.pose.position.y,
                                   msg.detections[0].pose.pose.pose.position.z);

    tf_tag2cam_.matrix().block<3, 1>(0, 3) = vec_tag2cam;
    tf_tag2cam_.matrix().block<3, 3>(0, 0) = rot_tag2cam.toRotationMatrix();

    Eigen::Affine3d tf_tag2world;
    tf_tag2world = tf_cam2base_ * tf_tag2cam_;

    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp            = ros::Time::now();
    pose_msg.header.frame_id         = "map";
    pose_msg.pose.pose.position.x    = tf_tag2world.matrix()(0, 3);
    pose_msg.pose.pose.position.y    = tf_tag2world.matrix()(1, 3);
    pose_msg.pose.pose.position.z    = tf_tag2world.matrix()(2, 3);
    pose_msg.pose.pose.orientation.w = rot_tag2cam.w();
    pose_msg.pose.pose.orientation.x = rot_tag2cam.x();
    pose_msg.pose.pose.orientation.y = rot_tag2cam.y();
    pose_msg.pose.pose.orientation.z = rot_tag2cam.z();
    pose_pub_.publish(pose_msg);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tag2pose");
  ros::NodeHandle nh("~");

  /** subscribe tag detection msg */
  ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 1, tagCallback);

  /** publish tag pose msg */
  pose_pub_ = nh.advertise<nav_msgs::Odometry>("/tag_odom", 1);

  /** load tf_cam2base from yaml file */
  std::pair<bool, Eigen::Affine3d> tf_cam2base_pair =
      dithas::utils::readTransform("/body_T_cam0/data");
  if (tf_cam2base_pair.first) {
    tf_cam2base_ = tf_cam2base_pair.second;
  } else {
    ROS_ERROR("Failed to load body_T_cam0.");
    tf_cam2base_ = Eigen::Affine3d::Identity();
  }

  /* initialize */

  ros::spin();
  return 0;
}
