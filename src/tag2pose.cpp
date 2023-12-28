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
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>

ros::Publisher pose_pub_;

Eigen::Affine3d tf_tag2cam_;

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

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp       = ros::Time::now();
    pose_msg.header.frame_id    = "map";
    pose_msg.pose.position.x    = tf_tag2cam_.matrix()(0, 3);
    pose_msg.pose.position.y    = tf_tag2cam_.matrix()(1, 3);
    pose_msg.pose.position.z    = tf_tag2cam_.matrix()(2, 3);
    pose_msg.pose.orientation.w = rot_tag2cam.w();
    pose_msg.pose.orientation.x = rot_tag2cam.x();
    pose_msg.pose.orientation.y = rot_tag2cam.y();
    pose_msg.pose.orientation.z = rot_tag2cam.z();
    pose_pub_.publish(pose_msg);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tag2pose");
  ros::NodeHandle nh("~");

  ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 1, tagCallback);

  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/tag_pose", 1);

  /* initialize */
  tf_tag2cam_ = Eigen::Affine3d::Identity();

  ros::spin();
  return 0;
}
