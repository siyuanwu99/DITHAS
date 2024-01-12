/**
 * @file
 * @brief
 *   This node will subscribe apriltag detection msg from apriltag node and convert to
 * nav_msgs/Path msg in local frame.
 *
 * @date 29/12/2023
 * @version 1.0.0
 * @Copyright 2023 <siyuanwu99@gmail.com>
 */

#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <dithas/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>

ros::Publisher path_pub_;
ros::Publisher pose_pub_;

Eigen::Isometry3d tf_tag2cam_  = Eigen::Isometry3d::Identity();
Eigen::Isometry3d tf_cam2base_ = Eigen::Isometry3d::Identity();

nav_msgs::Path path_msg_;
double         t0_;

void tagCallback(const apriltag_ros::AprilTagDetectionArray &msg) {
  static int idx = 0;
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

    Eigen::Isometry3d tf_tag2world;
    tf_tag2world                     = tf_cam2base_ * tf_tag2cam_;
    Eigen::Quaterniond rot_tag2world = Eigen::Quaterniond(tf_tag2world.matrix().block<3, 3>(0, 0));

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp       = ros::Time::now();
    pose_msg.header.frame_id    = "map";
    pose_msg.pose.position.x    = tf_tag2world.matrix()(0, 3);
    pose_msg.pose.position.y    = tf_tag2world.matrix()(1, 3);
    pose_msg.pose.position.z    = tf_tag2world.matrix()(2, 3);
    pose_msg.pose.orientation.w = rot_tag2world.w();
    pose_msg.pose.orientation.x = rot_tag2world.x();
    pose_msg.pose.orientation.y = rot_tag2world.y();
    pose_msg.pose.orientation.z = rot_tag2world.z();
    pose_pub_.publish(pose_msg);

    path_msg_.header.stamp    = ros::Time::now();
    path_msg_.header.frame_id = "map";
    path_msg_.poses.push_back(pose_msg);
    path_pub_.publish(path_msg_);
    idx++;
    ROS_INFO("Pose[%d]: %f, %f, %f, %f", idx, pose_msg.pose.position.x, pose_msg.pose.position.y,
             pose_msg.pose.position.z, pose_msg.header.stamp.toSec() - t0_);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tag2path");
  ros::NodeHandle nh("~");

  /** subscribe tag detection msg */
  ros::Subscriber tag_sub = nh.subscribe("tag_detections", 1, tagCallback);

  /** publish tag pose msg */
  path_pub_ = nh.advertise<nav_msgs::Path>("tag_path", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("tag_pose", 1);

  /** load tf_cam2base from yaml file */
  std::pair<bool, Eigen::Isometry3d> tf_cam2base_pair = dithas::readTransform("/body_T_cam0/data");
  if (tf_cam2base_pair.first) {
    tf_cam2base_ = tf_cam2base_pair.second;
  } else {
    ROS_ERROR("Failed to load body_T_cam0.");
    tf_cam2base_ = Eigen::Isometry3d::Identity();
  }

  /* initialize */
  path_msg_.header.stamp    = ros::Time::now();
  path_msg_.header.frame_id = "map";
  t0_                       = ros::Time::now().toSec();

  ros::spin();
  return 0;
}
