/**
 * @file lidar_plane_seg.cpp
 * @brief
 *
 *  This node will subscribe point cloud msg from livox and segment the plane in the point cloud.
 *
 * @date 2024-01-11
 * @version 1.0.0
 * @Copyright 2024 <siyuanwu99@gmail.com>
 */

#include <dithas/lidar_common.h>
#include <dithas/utils.h>
#include <nav_msgs/Odometry.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <backward.hpp>  // debug
#include <dithas/dbscan.hpp>
#include <iostream>
#include <queue>
#include <string>

/* Gloable Variables */

ros::Publisher plane_pub_;
ros::Publisher selected_plane_pub_;
ros::Publisher color_plane_pub_;
ros::Publisher pose_pub_;

/* Global parameters */
int   plane_max_num_(5);
int   plane_min_size_(10);
float board_size_(0.1);
float voxel_size_;
float theta_min_;
float theta_max_;
float min_line_dis_threshold_;
float max_line_dis_threshold_;

/** parameters for RANSAC Plane Extraction */
float ransac_dis_thres_(0.05);
int   plane_size_thres_(10);

/** parameters for Euclidean Clustering */
float clustering_tolerance_(0.1);
int   clustering_min_size_(20);

/**
 * @class PlaneComparator
 * @brief comparator for ranking the extracted planes
 */
struct PlaneComparator {
  bool operator()(const SinglePlane& lhs, const SinglePlane& rhs) const {
    /** Define the function to compare the plane */

    /** lambda function to calculate the distance between center to zero */
    auto dist_to_zero = [](const pcl::PointXYZ& p) {
      return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    };

    /** lambda function to evalute the size similarity to the template board size */
    auto cost_to_tmp = [](const SinglePlane& plane, float board_size) {

    };  // TODO: filtering w.r.t. size of the plane

    /** we would like the plane with lower cost */
    return dist_to_zero(lhs.p_center) > dist_to_zero(rhs.p_center); /** we use dist to zero here */
  }
};

pcl::PointCloud<PointType>::Ptr preprocessing(const pcl::PointCloud<PointType>::Ptr& lidar_cloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);

  /** preprocess filter out points far away from the lidar */
  pcl::PassThrough<pcl::PointXYZI> pass_x;
  pass_x.setInputCloud(lidar_cloud);
  pass_x.setFilterFieldName("z");
  pass_x.setFilterLimits(0.0, 2.5);
  pass_x.filter(*cloud_filtered);

  cloud_tmp.swap(cloud_filtered);
  pcl::PassThrough<pcl::PointXYZI> pass_z;
  pass_z.setInputCloud(cloud_tmp);
  pass_z.setFilterFieldName("x");
  pass_z.setFilterLimits(-1.0, 5.0);
  pass_z.filter(*cloud_filtered);

  cloud_tmp.swap(cloud_filtered);
  pcl::PassThrough<pcl::PointXYZI> pass_y;
  pass_y.setInputCloud(cloud_tmp);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-3.0, 3.0);
  pass_y.filter(*cloud_filtered);

  return cloud_filtered;
}

/**
 * @brief [TODO:description]
 * @param lidar_cloud [TODO:parameter]
 */
template <class T>
void extractLiDARPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_cloud, T& plane_lists) {
  ROS_INFO_STREAM("Extracting Lidar Edge");

  /** create a KD-tree object for efficient search */
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(lidar_cloud);

  /* cluster the point cloud */
  std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;  // Cluster by Euclidean Distance
  // ec.setClusterTolerance(clustering_tolerance_);
  // ec.setMinClusterSize(clustering_min_size_);  // Minimum size of a cluster
  // ec.setMaxClusterSize(25000);                 // Maximum size of a cluster
  // ec.setSearchMethod(tree);
  // ec.setInputCloud(lidar_cloud);
  // ec.extract(cluster_indices);

  DBSCANKdtreeCluster<pcl::PointXYZI> dbscan;  // Cluster by DBSCAN
  dbscan.setCorePointMinPts(30);
  dbscan.setClusterTolerance(clustering_tolerance_);
  dbscan.setMinClusterSize(clustering_min_size_);  // Minimum size of a cluster
  dbscan.setMaxClusterSize(25000);                 // Maximum size of a cluster
  dbscan.setSearchMethod(tree);
  dbscan.setInputCloud(lidar_cloud);
  dbscan.extract(cluster_indices);

  /** initialize the plane extraction */
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  // inliers表示误差能容忍的点，记录点云序号

  pcl::SACSegmentation<pcl::PointXYZI> seg;  // 创建一个分割器
  seg.setOptimizeCoefficients(true);  // Optional,设置结果平面展示的点是分割掉的点还是分割剩下的点
  seg.setModelType(pcl::SACMODEL_PLANE);        // Mandatory-设置目标几何形状
  seg.setMethodType(pcl::SAC_RANSAC);           // 分割方法：随机采样法
  seg.setDistanceThreshold(ransac_dis_thres_);  // 设置误差容忍范围，也就是阈值

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

  int cluster_id = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& idx : it->indices) {
      cloud_cluster->push_back((*lidar_cloud)[idx]);
    }
    cloud_cluster->width    = cloud_cluster->size();
    cloud_cluster->height   = 1;
    cloud_cluster->is_dense = true;

    /* extract plane from each cluster */
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    seg.setInputCloud(cloud_cluster); /** Input point cloud */
    seg.setMaxIterations(500);
    seg.segment(*inliers, *coefficients); /**  segment plane from point cloud */
    if (inliers->indices.size() == 0) {
      ROS_INFO_STREAM("Could not estimate a planner model for the given dataset");
      break;
    }

    extract.setIndices(inliers);
    extract.setInputCloud(cloud_cluster);
    extract.filter(*cloud_cluster);

    ROS_INFO("extract plane [%d] extract planner_cloud size: %d", cluster_id,
             (int)cloud_cluster->size());

    /** Colorize planes */
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;

    dithas::RGBColor color; /** generate random color */
    dithas::generateRandomRGBColor(color);

    pcl::PointXYZ p_center(0, 0, 0);
    for (size_t i = 0; i < cloud_cluster->points.size(); i++) {
      pcl::PointXYZRGB p;
      p.x = cloud_cluster->points[i].x;
      p.y = cloud_cluster->points[i].y;
      p.z = cloud_cluster->points[i].z;
      p_center.x += p.x;
      p_center.y += p.y;
      p_center.z += p.z;
      p.r = color[0];
      p.g = color[1];
      p.b = color[2];
      color_cloud.push_back(p);
      color_cloud_cluster->push_back(p);
    }
    p_center.x = p_center.x / cloud_cluster->size();
    p_center.y = p_center.y / cloud_cluster->size();
    p_center.z = p_center.z / cloud_cluster->size();

    /** Save plane */
    SinglePlane single_plane;
    single_plane.index    = cluster_id;
    single_plane.cloud    = *cloud_cluster;
    single_plane.p_center = p_center;
    single_plane.normal << coefficients->values[0], coefficients->values[1],
        coefficients->values[2];

    std::pair<double, double> width_height =
        getWidthHeight(cloud_cluster, p_center, single_plane.normal);
    single_plane.width  = width_height.first;
    single_plane.height = width_height.second;

    if (single_plane.width < 0.5 * board_size_ || single_plane.height < 0.5 * board_size_ ||
        single_plane.width > 2.5 * board_size_ || single_plane.height > 2.5 * board_size_) {
      continue;
    }

    // std::cout << "Getting plane at: " << single_plane.p_center
    //           << " with normal: " << single_plane.normal.transpose()
    //           << " width: " << single_plane.width << " height: " << single_plane.height
    //           << std::endl;

    plane_lists.push(single_plane);
    // plane_lists.push_back(single_plane);
    cluster_id++;
  }

  if (plane_lists.size() >= 1) { /** Publish segmented plane */
    sensor_msgs::PointCloud2 dbg_msg;
    pcl::toROSMsg(*color_cloud_cluster, dbg_msg);
    dbg_msg.header.frame_id = "camera_init";
    plane_pub_.publish(dbg_msg);
  }
}

void filterPLane(const std::vector<SinglePlane>& plane_lists, float board_size) {
  double min_cost = std::numeric_limits<double>::max();
  return;
}

void visualizePlane(const SinglePlane& plane) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "camera_init";
  marker.header.stamp    = ros::Time::now();

  marker.id     = 0;
  marker.type   = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = plane.p_center.x;
  marker.pose.position.y = plane.p_center.y;
  marker.pose.position.z = plane.p_center.z;

  // Use normal to calculate the orientation
  Eigen::Vector3d    rot_axis = Eigen::Vector3d::UnitZ().cross(plane.normal);
  double             angle    = acos(Eigen::Vector3d::UnitZ().dot(plane.normal));
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, rot_axis));

  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  marker.scale.x = 2 * plane.width;
  marker.scale.y = 2 * plane.height;
  marker.scale.z = 0.01;

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  // marker.lifetime = ros::Duration(1.0);
  selected_plane_pub_.publish(marker);
}

/**
 * @brief lidar callback function with all the processing
 * @param msg point cloud msg from lidar
 */
void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  // debug
  std::cout << "cloud size: " << cloud->size() << std::endl;
  std::cout << "cloud width: " << cloud->width << std::endl;
  std::cout << "cloud height: " << cloud->height << std::endl;
  std::cout << "cloud is_dense: " << cloud->is_dense << std::endl;
  std::cout << "cloud is organized: " << cloud->isOrganized() << std::endl;
  std::cout << "cloud point size: " << cloud->points.size() << std::endl;
  std::cout << "cloud sensor origin: " << cloud->sensor_origin_ << std::endl;
  std::cout << "cloud header frame id: " << cloud->header.frame_id << std::endl;
  std::cout << "cloud header seq: " << cloud->header.seq << std::endl;
  std::cout << "cloud header stamp: " << cloud->header.stamp << std::endl;

  /** 1. preprocess the point cloud */
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_processed = preprocessing(cloud);

  ros::Time start = ros::Time::now();
  /** 2. extract plane from point cloud */
  std::priority_queue<SinglePlane, std::vector<SinglePlane>, PlaneComparator> plane_lists;
  extractLiDARPlane(cloud_processed, plane_lists);

  if (plane_lists.empty()) {
    ROS_INFO_STREAM("No plane extracted");
    return;
  }
  ROS_INFO_STREAM("Extracting Lidar Plane Time: " << (ros::Time::now() - start).toSec() * 1000
                                                  << " ms");

  start = ros::Time::now();
  /** 3. filter out the desired board and calculate the center */
  // filterPlanes(plane_lists, board_size_);
  SinglePlane selected_plane = plane_lists.top();
  visualizePlane(selected_plane);
  ROS_INFO_STREAM("Filtering Lidar Plane Time: " << (ros::Time::now() - start).toSec() * 1000
                                                 << " ms");

  /** 3. EKF track board trajectories */

  /** 4. publish the center of the plane */
  nav_msgs::Odometry pose_msg;
  pose_msg.header.frame_id         = "map";
  pose_msg.header.stamp            = ros::Time::now();
  pose_msg.pose.pose.position.x    = selected_plane.p_center.x;
  pose_msg.pose.pose.position.y    = selected_plane.p_center.y;
  pose_msg.pose.pose.position.z    = selected_plane.p_center.z;
  pose_msg.pose.pose.orientation.w = 1.0;

  Eigen::Vector3d    rot_axis = Eigen::Vector3d::UnitX().cross(selected_plane.normal);
  double             angle    = acos(Eigen::Vector3d::UnitX().dot(selected_plane.normal));
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, rot_axis));
  pose_msg.pose.pose.orientation.x = q.x();
  pose_msg.pose.pose.orientation.y = q.y();
  pose_msg.pose.pose.orientation.z = q.z();
  pose_msg.pose.pose.orientation.w = q.w();
  pose_pub_.publish(pose_msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "lidar_plane_seg");
  ros::NodeHandle nh("~");

  nh.param<float>("voxel_size", voxel_size_, 1);
  nh.param<float>("theta_min", theta_min_, 0.9);
  nh.param<float>("theta_max", theta_max_, 0.99);
  nh.param<float>("board_size", board_size_, 0.1);
  nh.param<float>("clustering_tolerance", clustering_tolerance_, 0.1);
  nh.param<int>("clustering_min_size", clustering_min_size_, 20);
  nh.param<float>("min_line_dis_threshold", min_line_dis_threshold_, 0.01);
  nh.param<float>("max_line_dis_threshold", max_line_dis_threshold_, 0.1);
  nh.param<int>("plane_max_num", plane_max_num_, 5);
  nh.param<int>("plane_min_size", plane_min_size_, 10);
  nh.param<float>("ransac_dis_thres", ransac_dis_thres_, 0.1);

  ros::Subscriber sub = nh.subscribe("point_cloud", 1, lidarCallback);

  plane_pub_       = nh.advertise<sensor_msgs::PointCloud2>("/voxel_plane", 100);
  pose_pub_        = nh.advertise<nav_msgs::Odometry>("plane_odom", 100);
  color_plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/color_cloud", 100);

  selected_plane_pub_ = nh.advertise<visualization_msgs::Marker>("/selected_plane", 100);

  ros::spin();
  return 0;
}
