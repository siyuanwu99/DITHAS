
/**
 * @file consensus_solver.cpp
 * @brief
 *   1. maintain a aligned point pair buffer
 *   2. use BA to jointly optimize the tf and global path
 *   3. Share optimized path with other robots
 *
 *
 * @date 2024-01-02
 * @version 1.0.0
 * @Copyright 2024 <siyuanwu99@gmail.com>
 */

#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <dithas/se3_solver.h>
#include <dithas/utils.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
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

const int    BUFFER_SIZE = 500;
const double SOLVE_TIME  = 0.2;

/**
 * @struct TrajPoint
 * @brief Poses in trajectory
 */
struct TrajPoint {
  /** default constructor */
  TrajPoint() {
    pos_  = Eigen::Vector3d::Zero();
    time_ = 0.0;
  }
  TrajPoint(const Eigen::Vector3d& pos, const double time) : pos_(pos), time_(time) {}
  Eigen::Vector3d pos_;
  double          time_;
};

struct TrajLinkedNode {
  TrajLinkedNode() {
    pos       = Eigen::Vector3d::Zero();
    timestamp = 0.0;
    next      = nullptr;
    prev      = nullptr;
  }
  TrajLinkedNode(const Eigen::Vector3d& pos, const double time)
      : pos(pos), timestamp(time), next(nullptr), prev(nullptr) {}
  explicit TrajLinkedNode(const double time) : timestamp(time) {
    pos  = Eigen::Vector3d::Zero();
    next = nullptr;
    prev = nullptr;
  }
  Eigen::Vector3d pos;
  double          timestamp;
  TrajLinkedNode* next;
  TrajLinkedNode* prev;
};

/**
 * @class TrajList
 * @brief Linked list for trajectory
 *
 */
class TrajList {
 public:
  /* constructor */
  TrajList() {
    size             = 0;
    dummy_head       = new TrajLinkedNode();
    dummy_tail       = new TrajLinkedNode();
    dummy_head->next = dummy_tail;
    dummy_tail->prev = dummy_head;
  }
  explicit TrajList(const double t) {
    size             = 0;
    dummy_head       = new TrajLinkedNode(t);
    dummy_tail       = new TrajLinkedNode(t);
    dummy_head->next = dummy_tail;
    dummy_tail->prev = dummy_head;
  }
  ~TrajList() {
    delete dummy_head;
    std::cout << "Delete TrajList" << std::endl;
  }

  void addToEnd(const Eigen::Vector3d& pos, const double time) {
    TrajLinkedNode* new_node = new TrajLinkedNode(pos, time);
    dummy_tail->prev->next   = new_node;
    dummy_tail->prev         = new_node;
  }

  void printAll() {
    TrajLinkedNode* cur = dummy_head->next;
    while (cur != dummy_tail) {
      std::cout << std::fixed << std::setprecision(3) << "t: " << cur->timestamp
                << " p: " << cur->pos.transpose() << std::endl;
      cur = cur->next;
    }
  }

  TrajLinkedNode* getNearest(const double time) {
    TrajLinkedNode* cur = dummy_tail->prev;
    while (cur != dummy_head) {
      if (cur->timestamp <= time) {
        double to_prev = std::abs(cur->timestamp - time);
        double to_next = std::abs(cur->next->timestamp - time);
        if (to_prev < to_next) {
          return cur;
        } else {
          return cur->next;
        }
      }
      cur = cur->prev;
    }
    return nullptr;
  }

  TrajLinkedNode* getHead() const { return dummy_head->next; }
  TrajLinkedNode* getTail() const { return dummy_tail->prev; }

 protected:
  int             size;
  TrajLinkedNode* dummy_head;
  TrajLinkedNode* dummy_tail;
};

typedef std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> PointPairs;
typedef std::vector<Eigen::Vector3d>                             Points;
typedef std::vector<TrajLinkedNode>                              ObsrvsLinked;

/** publisher */
ros::Publisher pt_pub_, path_pub_, odom_pub_;

Eigen::Isometry3d tf_tag2cam_  = Eigen::Isometry3d::Identity(); /** tf from tag to camera */
Eigen::Isometry3d tf_cam2base_ = Eigen::Isometry3d::Identity(); /** tf from camera to body */

SE3d tf_self2glbl_(Eigen::Matrix3d::Identity(),
                   Eigen::Vector3d::Zero()); /** tf from self to global */

/** flag */
int solve_index_(0);

/** observations */
ObsrvsLinked buf_obsrv_1_;
ObsrvsLinked buf_obsrv_2_;
ObsrvsLinked buf_obsrv_3_;

/**
 * @brief obsrvCallback which save the latest obsrv msg into buffer
 *
 * @param buf buffer to save the latest obsrv msger
 * @param msg observation msg
 */
void obsrvCallback(std::vector<TrajPoint>& buf, const geometry_msgs::PointStamped& msg) {
  TrajPoint traj_point(Eigen::Vector3d(msg.point.x, msg.point.y, msg.point.z),
                       msg.header.stamp.toSec());
  if (buf.size() <= BUFFER_SIZE) {
    buf.push_back(traj_point);
  } else {
    buf.erase(buf.begin());
    buf.push_back(traj_point);
  }
}

void obsrv1Callback(const geometry_msgs::PointStamped& msg) { obsrvCallback(buf_obsrv_self_, msg); }
void obsrv2Callback(const geometry_msgs::PointStamped& msg) { obsrvCallback(buf_obsrv_glbl_, msg); }
void obsrv3Callback(const geometry_msgs::PointStamped& msg) { obsrvCallback(buf_obsrv_glbl_, msg); }

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

  /* publish odometry msg */
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
  PointPairs aligned_points = getPointPairs(buf_obsrv_1_, buf_obsrv_2_);
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
  ros::init(argc, argv, "consensus_solver");
  ros::NodeHandle nh("~");

  /** subscribe tag detection odom msg */
  ros::Subscriber obsrv1_sub = nh.subscribe("obsrv1", 1, obsrv1Callback);
  ros::Subscriber obsrv2_sub = nh.subscribe("obsrv2", 1, obsrv2Callback);
  ros::Subscriber obsrv3_sub = nh.subscribe("obsrv3", 1, obsrv3Callback);

  /** publish transform msg */
  // pt_pub_   = nh.advertise<geometry_msgs::PointStamped>("global/point", 1);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("global/odom", 1);
  path_pub_ = nh.advertise<nav_msgs::Path>("opt_path", 1);

  /** Solve timer **/
  ros::Timer solve_timer = nh.createTimer(ros::Duration(SOLVE_TIME), solveCallback);

  /* initialize */
  buf_obsrv_1_.reserve(BUFFER_SIZE);
  buf_obsrv_2_.reserve(BUFFER_SIZE);
  buf_obsrv_3_.reserve(BUFFER_SIZE);

  ROS_INFO("Initialize pose solver.");
  ROS_INFO("tf_cam2base: \n %f, %f, %f, %f, %f, %f, %f", tf_cam2base_.matrix()(0, 0),
           tf_cam2base_.matrix()(0, 1), tf_cam2base_.matrix()(0, 2), tf_cam2base_.matrix()(0, 3),
           tf_cam2base_.matrix()(1, 0), tf_cam2base_.matrix()(1, 1), tf_cam2base_.matrix()(1, 2));

  ros::spin();
  return 0;
}
