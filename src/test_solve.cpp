/**
 * @file
 * @brief
 *
 * @date 30/12/2023
 * @version 1.0.0
 * @Copyright 2023 <siyuanwu99@gmail.com>
 */

#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <dithas/se3_solver.h>
#include <dithas/utils.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <backward.hpp>  // debug
#include <iostream>
#include <string>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

typedef Sophus::SE3d SE3d;

const int    BUFFER_SIZE = 20;
const double SOLVE_TIME  = 0.2;

/**
 * @struct TrajPoint
 * @brief Poses in trajectory
 */
struct TrajPoint {
  /** default constructor */
  TrajPoint() {
    pos_  = Eigen::Vector3d::Zero();
    rot_  = Eigen::Quaterniond::Identity();
    time_ = 0.0;
  }
  TrajPoint(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot, const double time)
      : pos_(pos), rot_(rot), time_(time) {}
  Eigen::Vector3d    pos_;
  Eigen::Quaterniond rot_;
  double             time_;
};

typedef std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> PointPairs;
typedef std::vector<Eigen::Vector3d>                             Points;

/** publisher */
ros::Publisher odom_pub_, path_pub_;

/** tf from tag to camera */
Eigen::Isometry3d tf_tag2cam_ = Eigen::Isometry3d::Identity();

/** tf from camera to body */
Eigen::Isometry3d tf_cam2base_ = Eigen::Isometry3d::Identity();

/** buffer for tag detections in body frame */
std::vector<TrajPoint> buf_odom_self_;
std::vector<TrajPoint> buf_odom_glbl_;

/** tf from self to global */
SE3d tf_self2glbl_(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

/** flag */
int solve_index_(0);

/**
 * @brief odomCallback which save the latest odom msg into buffer
 *
 * @param buf buffer to save the latest odom msger
 * @param msg odometry msg
 */
void odomCallback(std::vector<TrajPoint>& buf, const nav_msgs::Odometry& msg) {
  TrajPoint traj_point(
      Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
      Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                         msg.pose.pose.orientation.y, msg.pose.pose.orientation.z),
      msg.header.stamp.toSec());
  if (buf.size() <= BUFFER_SIZE) {
    buf.push_back(traj_point);
  } else {
    buf.erase(buf.begin());
    buf.push_back(traj_point);
  }
}

void odomSelfCallback(const nav_msgs::Odometry& msg) { odomCallback(buf_odom_self_, msg); }
void odomGlblCallback(const nav_msgs::Odometry& msg) { odomCallback(buf_odom_glbl_, msg); }

/**
 * @brief get aligned point pairs from two trajectories
 *
 * @param src source trajectory
 * @param dst destination trajectory
 * @return aligned point pairs w.r.t. time
 */
PointPairs getPointPairs(const std::vector<TrajPoint>& src, const std::vector<TrajPoint>& dst) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> alignment;

  int j = 0;
  for (size_t i = 0; i < src.size(); ++i) {
    const double MAX_TEMPORAL_DIFF = 0.050;
    double       min_diff          = MAX_TEMPORAL_DIFF;
    int          closest_idx       = -1;

    while (j < dst.size()) {
      double diff = std::abs(src[i].time_ - dst[j].time_);
      if (diff < min_diff) {
        min_diff    = diff;
        closest_idx = j;
      } else {
        break;
      }
      j++;
    }

    if (min_diff < MAX_TEMPORAL_DIFF) {
      alignment.push_back(std::make_pair(src[i].pos_, dst[closest_idx].pos_));
    } else {
      continue;
    }
  }

  return alignment;
}

/**
 * @brief convert point pairs to points
 *
 * @param pairs point pairs
 * @param src_points output source points
 * @param dst_points output destination points
 */
void convertPairsToPoints(const PointPairs& pairs, Points& src_points, Points& dst_points) {
  src_points.resize(pairs.size());
  dst_points.resize(pairs.size());
  for (size_t i = 0; i < pairs.size(); ++i) {
    src_points[i] = pairs[i].first;
    dst_points[i] = pairs[i].second;
  }
}

/**
 * @brief publish transformed trajectories in global frame
 *
 * @param pts points in local frame
 * @param tf transform from local to global
 */
void publishTransfromedTrajectories(const Points& pts, const Eigen::Isometry3d& tf) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp    = ros::Time::now();
  path_msg.header.frame_id = "map";

  for (size_t i = 0; i < pts.size(); ++i) {
    Eigen::Vector3d            pt = tf * pts[i];
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp    = ros::Time::now();
    pose_msg.pose.position.x = pt[0];
    pose_msg.pose.position.y = pt[1];
    pose_msg.pose.position.z = pt[2];
    Eigen::Quaterniond rot(tf.rotation());
    pose_msg.pose.orientation.w = rot.w();
    pose_msg.pose.orientation.x = rot.x();
    pose_msg.pose.orientation.y = rot.y();
    pose_msg.pose.orientation.z = rot.z();
    path_msg.poses.push_back(pose_msg);
  }
  path_pub_.publish(path_msg);

  ROS_INFO("publised tf: %f, %f, %f", tf.translation()[0], tf.translation()[1],
           tf.translation()[2]);
  Eigen::Quaterniond rot(tf.rotation());
  Eigen::Vector3d    pos = tf.translation();
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp            = ros::Time::now();
  odom_msg.header.frame_id         = "map";
  odom_msg.pose.pose.position.x    = pos.x();
  odom_msg.pose.pose.position.y    = pos.y();
  odom_msg.pose.pose.position.z    = pos.z();
  odom_msg.pose.pose.orientation.w = rot.w();
  odom_msg.pose.pose.orientation.x = rot.x();
  odom_msg.pose.pose.orientation.y = rot.y();
  odom_msg.pose.pose.orientation.z = rot.z();
  odom_msg.twist.twist.linear.x    = 0;
  odom_msg.twist.twist.linear.y    = 0;
  odom_msg.twist.twist.linear.z    = 1;

  odom_pub_.publish(odom_msg);
}

/**
 * @brief solveCallback which solve the tf from self to global
 * Description:
 * 1. align point pairs stored in the buffer
 * 2. solve the least square problem to get the transform from self to global
 * 3. publish tf and aligned points in global frame
 *
 * @param event
 */
void solveCallback(const ros::TimerEvent& event) {
  ROS_INFO("----- Solve %i -----", solve_index_);

  /** step 1: align point pairs stored in the buffer */
  PointPairs aligned_points = getPointPairs(buf_odom_self_, buf_odom_glbl_);
  Points     self_points, glbl_points;
  convertPairsToPoints(aligned_points, self_points, glbl_points);

  ROS_INFO("Aligned %lu points.", aligned_points.size());
  if (self_points.size() < 10) {
    ROS_WARN("Not enough points to solve tf, skip. (%lu aligned points)", self_points.size());
    return;
  }

  /** step 2: solve the tf from self to global */
  dithas::solver::SE3Solver solver;
  if (solver.solve(self_points, glbl_points)) {
    tf_self2glbl_ = solver.getSophusSE3d();
    ROS_INFO("Solved tf from self to global: \n %f, %f, %f, %f, %f, %f, %f",
             tf_self2glbl_.matrix()(0, 0), tf_self2glbl_.matrix()(0, 1),
             tf_self2glbl_.matrix()(0, 2), tf_self2glbl_.matrix()(0, 3),
             tf_self2glbl_.matrix()(1, 0), tf_self2glbl_.matrix()(1, 1),
             tf_self2glbl_.matrix()(1, 2));
    solve_index_++;
  } else {
    ROS_ERROR("Failed to solve tf from self to global.");
  }

  /** step 3: publish tf and aligned points in global frame */
  // Convert tf from Sophus::SE3d to Eigen::Isometry3d
  Eigen::Isometry3d tf_self2glbl = Eigen::Isometry3d::Identity();
  tf_self2glbl.rotate(tf_self2glbl_.rotationMatrix());
  tf_self2glbl.pretranslate(tf_self2glbl_.translation());
  publishTransfromedTrajectories(self_points, tf_self2glbl);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pose_solver");
  ros::NodeHandle nh("~");

  /** subscribe tag detection odom msg */
  ros::Subscriber odom1_sub = nh.subscribe("tag_odom_self", 1, odomSelfCallback);
  ros::Subscriber odom2_sub = nh.subscribe("tag_odom_glbl", 1, odomGlblCallback);

  /** publish transform msg */
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("rst_odom", 1);
  path_pub_ = nh.advertise<nav_msgs::Path>("rst_path", 1);

  /** Solve timer **/
  ros::Timer solve_timer = nh.createTimer(ros::Duration(SOLVE_TIME), solveCallback);

  /* initialize */
  buf_odom_self_.reserve(BUFFER_SIZE);
  buf_odom_glbl_.reserve(BUFFER_SIZE);

  ROS_INFO("Initialize pose solver.");
  ROS_INFO("tf_cam2base: \n %f, %f, %f, %f, %f, %f, %f", tf_cam2base_.matrix()(0, 0),
           tf_cam2base_.matrix()(0, 1), tf_cam2base_.matrix()(0, 2), tf_cam2base_.matrix()(0, 3),
           tf_cam2base_.matrix()(1, 0), tf_cam2base_.matrix()(1, 1), tf_cam2base_.matrix()(1, 2));

  ros::spin();
  return 0;
}
