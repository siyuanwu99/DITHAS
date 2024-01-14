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
#include <Eigen/Dense>
#include <backward.hpp>  // debug
#include <iostream>
#include <string>

/* Gloable Variables */

ros::Publisher plane_pub_;
ros::Publisher edge_pub_;
ros::Publisher color_plane_pub_;

pcl::PointCloud<pcl::PointXYZI>::Ptr edge_clouds_;  // edge clouds in global frame
std::vector<int>                     edge_counts_;

/* Global parameters */
int   edge_number_ = 0;
int   plane_max_num_(5);
int   plane_min_size_(10);
float voxel_size_;
float theta_min_;
float theta_max_;
float min_line_dis_threshold_;
float max_line_dis_threshold_;

float ransac_dis_thres_(0.05);
int   plane_size_thres_(10);

/**
 * @brief [TODO:description]
 * @param lidar_cloud [TODO:parameter]
 */
void extractLiDARPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_cloud) {
  ROS_INFO_STREAM("Extracting Lidar Edge");
  std::vector<SinglePlane>             plane_lists;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  /** preprocess filter out points far away from the lidar */
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI>     pass_x;
  pass_x.setInputCloud(lidar_cloud);
  pass_x.setFilterFieldName("z");
  pass_x.setFilterLimits(0.0, 2.5);
  pass_x.filter(*cloud_tmp);

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

  /** create a KD-tree object for efficient search */
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_filtered);

  /* cluster the point cloud by Euclidean Distance */
  std::vector<pcl::PointIndices>                  cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.10);
  ec.setMinClusterSize(20);     // Minimum size of a cluster
  ec.setMaxClusterSize(25000);  // Maximum size of a cluster
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

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
      cloud_cluster->push_back((*cloud_filtered)[idx]);
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

    dithas::RGBColor color;
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
    single_plane.cloud    = *cloud_cluster;
    single_plane.p_center = p_center;
    single_plane.normal << coefficients->values[0], coefficients->values[1],
        coefficients->values[2];
    single_plane.index = cluster_id;
    plane_lists.push_back(single_plane);
    cluster_id++;
  }
  if (plane_lists.size() >= 1) { /** Publish segmented plane */
    sensor_msgs::PointCloud2 dbg_msg;
    pcl::toROSMsg(*color_cloud_cluster, dbg_msg);
    dbg_msg.header.frame_id = "camera_init";
    plane_pub_.publish(dbg_msg);
  }
}

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

  std::unordered_map<VOXEL_LOC, Voxel*> voxel_map;

  ros::Time start = ros::Time::now();
  extractLiDARPlane(cloud);
  ROS_INFO_STREAM("Extracting Lidar Plane Time: " << (ros::Time::now() - start).toSec() * 1000
                                                  << " ms");
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "lidar_plane_seg");
  ros::NodeHandle nh("~");

  nh.param<float>("voxel_size", voxel_size_, 1);
  nh.param<float>("theta_min", theta_min_, 0.9);
  nh.param<float>("theta_max", theta_max_, 0.99);
  nh.param<float>("min_line_dis_threshold", min_line_dis_threshold_, 0.01);
  nh.param<float>("max_line_dis_threshold", max_line_dis_threshold_, 0.1);
  nh.param<int>("plane_max_num", plane_max_num_, 5);
  nh.param<int>("plane_min_size", plane_min_size_, 10);
  nh.param<float>("ransac_dis_thres", ransac_dis_thres_, 0.01);

  ros::Subscriber sub = nh.subscribe("point_cloud", 1, lidarCallback);

  plane_pub_       = nh.advertise<sensor_msgs::PointCloud2>("/voxel_plane", 100);
  edge_pub_        = nh.advertise<sensor_msgs::PointCloud2>("/lidar_edge", 100);
  color_plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/color_cloud", 100);

  ros::spin();
  return 0;
}
