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

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <backward.hpp>  // debug
#include <iostream>
#include <string>

class VOXEL_LOC {
 public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

typedef struct SinglePlane {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZ                   p_center;
  Eigen::Vector3d                 normal;
  int                             index;
} SinglePlane;

void LiDAREdgeExtraction(const std::unordered_map<VOXEL_LOC, Voxel*>& voxel_map,
                         const float                                  ransac_dis_thre,
                         const int                                    plane_size_threshold,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr&        lidar_edge_clouds_3d) {
  ROS_INFO_STREAM("Extracting Lidar Edge");
  ros::Rate loop(5000);
  lidar_edge_clouds_3d = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second->cloud->size() > 50) {
      std::vector<SinglePlane> plane_lists;
      // 创建一个体素滤波器
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::copyPointCloud(*iter->second->cloud, *cloud_filter);
      // 创建一个模型参数对象，用于记录结果
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      // inliers表示误差能容忍的点，记录点云序号
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      // 创建一个分割器
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      // Optional,设置结果平面展示的点是分割掉的点还是分割剩下的点
      seg.setOptimizeCoefficients(true);
      // Mandatory-设置目标几何形状
      seg.setModelType(pcl::SACMODEL_PLANE);
      // 分割方法：随机采样法
      seg.setMethodType(pcl::SAC_RANSAC);
      // 设置误差容忍范围，也就是阈值
      seg.setDistanceThreshold(ransac_dis_thre);
      pcl::PointCloud<pcl::PointXYZRGB> color_planner_cloud;
      int                               plane_index = 0;
      while (cloud_filter->points.size() > 10) {
        pcl::PointCloud<pcl::PointXYZI>     planner_cloud;
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        // 输入点云
        seg.setInputCloud(cloud_filter);
        seg.setMaxIterations(500);
        // 分割点云
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
          ROS_INFO_STREAM("Could not estimate a planner model for the given dataset");
          break;
        }
        extract.setIndices(inliers);
        extract.setInputCloud(cloud_filter);
        extract.filter(planner_cloud);

        if (planner_cloud.size() > plane_size_threshold) {
          pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
          std::vector<unsigned int>         colors;
          colors.push_back(static_cast<unsigned int>(rand() % 256));
          colors.push_back(static_cast<unsigned int>(rand() % 256));
          colors.push_back(static_cast<unsigned int>(rand() % 256));
          pcl::PointXYZ p_center(0, 0, 0);
          for (size_t i = 0; i < planner_cloud.points.size(); i++) {
            pcl::PointXYZRGB p;
            p.x = planner_cloud.points[i].x;
            p.y = planner_cloud.points[i].y;
            p.z = planner_cloud.points[i].z;
            p_center.x += p.x;
            p_center.y += p.y;
            p_center.z += p.z;
            p.r = colors[0];
            p.g = colors[1];
            p.b = colors[2];
            color_cloud.push_back(p);
            color_planner_cloud.push_back(p);
          }
          p_center.x = p_center.x / planner_cloud.size();
          p_center.y = p_center.y / planner_cloud.size();
          p_center.z = p_center.z / planner_cloud.size();
          SinglePlane single_plane;
          single_plane.cloud    = planner_cloud;
          single_plane.p_center = p_center;
          single_plane.normal << coefficients->values[0], coefficients->values[1],
              coefficients->values[2];
          single_plane.index = plane_index;
          plane_lists.push_back(single_plane);
          plane_index++;
        }
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZI> cloud_f;
        extract.filter(cloud_f);
        *cloud_filter = cloud_f;
      }
      if (plane_lists.size() >= 1) {
        sensor_msgs::PointCloud2 dbg_msg;
        pcl::toROSMsg(color_planner_cloud, dbg_msg);
        dbg_msg.header.frame_id = "camera_init";
        pub_plane.publish(dbg_msg);
        loop.sleep();
      }
      std::vector<pcl::PointCloud<pcl::PointXYZI>> edge_cloud_lists;
      calcLine(plane_lists, voxel_size_, iter->second->voxel_origin, edge_cloud_lists);
      if (edge_cloud_lists.size() > 0 && edge_cloud_lists.size() <= 5)
        for (size_t a = 0; a < edge_cloud_lists.size(); a++) {
          for (size_t i = 0; i < edge_cloud_lists[a].size(); i++) {
            pcl::PointXYZI p = edge_cloud_lists[a].points[i];
            lidar_edge_clouds->points.push_back(p);
            lidar_edge_numbers.push_back(edge_number_);
          }
          sensor_msgs::PointCloud2 dbg_msg;
          pcl::toROSMsg(edge_cloud_lists[a], dbg_msg);
          dbg_msg.header.frame_id = "camera_init";
          pub_edge.publish(dbg_msg);
          loop.sleep();
          edge_number_++;
        }
    }
  }
}

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "lidar_plane_seg");
  ros::NodeHandle nh("~");

  ros::Subscriber sub = nh.subscribe("point_cloud", 1, lidarCallback);

  ros::spin();
  return 0;
}
