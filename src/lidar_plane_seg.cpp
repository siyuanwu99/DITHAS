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
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
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
int   edge_number_    = 0;
int   plane_max_size_ = 5;
float voxel_size_;
float theta_min_;
float theta_max_;
float min_line_dis_threshold_;
float max_line_dis_threshold_;

/**
 * @brief [TODO:description]
 *
 * @param plane_lists [TODO:parameter]
 * @param voxel_size [TODO:parameter]
 * @param origin [TODO:parameter]
 * @param edge_cloud_lists [TODO:parameter]
 */
void calcLine(const std::vector<SinglePlane>&               plane_lists,
              const double                                  voxel_size,
              const Eigen::Vector3d                         origin,
              std::vector<pcl::PointCloud<pcl::PointXYZI>>& edge_cloud_lists) {
  if (plane_lists.size() >= 2 && plane_lists.size() <= plane_max_size_) {
    pcl::PointCloud<pcl::PointXYZI> temp_line_cloud;
    for (size_t plane_idx1 = 0; plane_idx1 < plane_lists.size() - 1; plane_idx1++) {
      for (size_t plane_idx2 = plane_idx1 + 1; plane_idx2 < plane_lists.size(); plane_idx2++) {
        float a1                  = plane_lists[plane_idx1].normal[0];
        float b1                  = plane_lists[plane_idx1].normal[1];
        float c1                  = plane_lists[plane_idx1].normal[2];
        float x1                  = plane_lists[plane_idx1].p_center.x;
        float y1                  = plane_lists[plane_idx1].p_center.y;
        float z1                  = plane_lists[plane_idx1].p_center.z;
        float a2                  = plane_lists[plane_idx2].normal[0];
        float b2                  = plane_lists[plane_idx2].normal[1];
        float c2                  = plane_lists[plane_idx2].normal[2];
        float x2                  = plane_lists[plane_idx2].p_center.x;
        float y2                  = plane_lists[plane_idx2].p_center.y;
        float z2                  = plane_lists[plane_idx2].p_center.z;
        float theta               = a1 * a2 + b1 * b2 + c1 * c2;
        float point_dis_threshold = 0.00;
        if (theta > theta_max_ && theta < theta_min_) {
          if (plane_lists[plane_idx1].cloud.size() > 0 ||
              plane_lists[plane_idx2].cloud.size() > 0) {
            float matrix[4][5];
            matrix[1][1] = a1;
            matrix[1][2] = b1;
            matrix[1][3] = c1;
            matrix[1][4] = a1 * x1 + b1 * y1 + c1 * z1;
            matrix[2][1] = a2;
            matrix[2][2] = b2;
            matrix[2][3] = c2;
            matrix[2][4] = a2 * x2 + b2 * y2 + c2 * z2;

            std::vector<Eigen::Vector3d> points;
            Eigen::Vector3d              point;
            matrix[3][1] = 1;
            matrix[3][2] = 0;
            matrix[3][3] = 0;
            matrix[3][4] = origin[0];
            calc<float>(matrix, point);
            if (point[0] >= origin[0] - point_dis_threshold &&
                point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                point[1] >= origin[1] - point_dis_threshold &&
                point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                point[2] >= origin[2] - point_dis_threshold &&
                point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

            matrix[3][1] = 0;
            matrix[3][2] = 1;
            matrix[3][3] = 0;
            matrix[3][4] = origin[1];
            calc<float>(matrix, point);
            if (point[0] >= origin[0] - point_dis_threshold &&
                point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                point[1] >= origin[1] - point_dis_threshold &&
                point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                point[2] >= origin[2] - point_dis_threshold &&
                point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

            matrix[3][1] = 0;
            matrix[3][2] = 0;
            matrix[3][3] = 1;
            matrix[3][4] = origin[2];
            calc<float>(matrix, point);
            if (point[0] >= origin[0] - point_dis_threshold &&
                point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                point[1] >= origin[1] - point_dis_threshold &&
                point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                point[2] >= origin[2] - point_dis_threshold &&
                point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

            matrix[3][1] = 1;
            matrix[3][2] = 0;
            matrix[3][3] = 0;
            matrix[3][4] = origin[0] + voxel_size;
            calc<float>(matrix, point);
            if (point[0] >= origin[0] - point_dis_threshold &&
                point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                point[1] >= origin[1] - point_dis_threshold &&
                point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                point[2] >= origin[2] - point_dis_threshold &&
                point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

            matrix[3][1] = 0;
            matrix[3][2] = 1;
            matrix[3][3] = 0;
            matrix[3][4] = origin[1] + voxel_size;
            calc<float>(matrix, point);
            if (point[0] >= origin[0] - point_dis_threshold &&
                point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                point[1] >= origin[1] - point_dis_threshold &&
                point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                point[2] >= origin[2] - point_dis_threshold &&
                point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

            matrix[3][1] = 0;
            matrix[3][2] = 0;
            matrix[3][3] = 1;
            matrix[3][4] = origin[2] + voxel_size;
            calc<float>(matrix, point);
            if (point[0] >= origin[0] - point_dis_threshold &&
                point[0] <= origin[0] + voxel_size + point_dis_threshold &&
                point[1] >= origin[1] - point_dis_threshold &&
                point[1] <= origin[1] + voxel_size + point_dis_threshold &&
                point[2] >= origin[2] - point_dis_threshold &&
                point[2] <= origin[2] + voxel_size + point_dis_threshold)
              points.push_back(point);

            if (points.size() == 2) {
              pcl::PointCloud<pcl::PointXYZI> edge_clouds;
              pcl::PointXYZ                   p1(points[0][0], points[0][1], points[0][2]);
              pcl::PointXYZ                   p2(points[1][0], points[1][1], points[1][2]);
              float length = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
              // 指定近邻个数
              int K = 1;
              // 创建两个向量，分别存放近邻的索引值、近邻的中心距
              std::vector<int>                         pointIdxNKNSearch1(K);
              std::vector<float>                       pointNKNSquaredDistance1(K);
              std::vector<int>                         pointIdxNKNSearch2(K);
              std::vector<float>                       pointNKNSquaredDistance2(K);
              pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree1(
                  new pcl::search::KdTree<pcl::PointXYZI>());
              pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree2(
                  new pcl::search::KdTree<pcl::PointXYZI>());
              kdtree1->setInputCloud(plane_lists[plane_idx1].cloud.makeShared());
              kdtree2->setInputCloud(plane_lists[plane_idx2].cloud.makeShared());
              for (float inc = 0; inc <= length; inc += 0.01) {
                pcl::PointXYZI p;
                p.x         = p1.x + (p2.x - p1.x) * inc / length;
                p.y         = p1.y + (p2.y - p1.y) * inc / length;
                p.z         = p1.z + (p2.z - p1.z) * inc / length;
                p.intensity = 100;
                if ((kdtree1->nearestKSearch(p, K, pointIdxNKNSearch1, pointNKNSquaredDistance1) >
                     0) &&
                    (kdtree2->nearestKSearch(p, K, pointIdxNKNSearch2, pointNKNSquaredDistance2) >
                     0)) {
                  float dis1 =
                      pow(p.x - plane_lists[plane_idx1].cloud.points[pointIdxNKNSearch1[0]].x, 2) +
                      pow(p.y - plane_lists[plane_idx1].cloud.points[pointIdxNKNSearch1[0]].y, 2) +
                      pow(p.z - plane_lists[plane_idx1].cloud.points[pointIdxNKNSearch1[0]].z, 2);
                  float dis2 =
                      pow(p.x - plane_lists[plane_idx2].cloud.points[pointIdxNKNSearch2[0]].x, 2) +
                      pow(p.y - plane_lists[plane_idx2].cloud.points[pointIdxNKNSearch2[0]].y, 2) +
                      pow(p.z - plane_lists[plane_idx2].cloud.points[pointIdxNKNSearch2[0]].z, 2);
                  if ((dis1 < min_line_dis_threshold_ * min_line_dis_threshold_ &&
                       dis2 < max_line_dis_threshold_ * max_line_dis_threshold_) ||
                      ((dis1 < max_line_dis_threshold_ * max_line_dis_threshold_ &&
                        dis2 < min_line_dis_threshold_ * min_line_dis_threshold_)))
                    edge_clouds.push_back(p);
                }
              }
              if (edge_clouds.size() > 30) edge_cloud_lists.push_back(edge_clouds);
            }
          }
        }
      }
    }
  }
}

/**
 * @brief extract the edge from the plane
 *
 * @param voxel_map [TODO:parameter]
 * @param ransac_dis_thre [TODO:parameter]
 * @param plane_size_threshold [TODO:parameter]
 * @param lidar_edge_clouds_3d [TODO:parameter]
 */
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
        plane_pub_.publish(dbg_msg);
        loop.sleep();
      }
      std::vector<pcl::PointCloud<pcl::PointXYZI>> edge_cloud_lists;
      calcLine(plane_lists, voxel_size_, iter->second->voxel_origin, edge_cloud_lists);
      if (edge_cloud_lists.size() > 0 && edge_cloud_lists.size() <= 5)
        for (size_t a = 0; a < edge_cloud_lists.size(); a++) {
          for (size_t i = 0; i < edge_cloud_lists[a].size(); i++) {
            pcl::PointXYZI p = edge_cloud_lists[a].points[i];
            edge_clouds_->points.push_back(p);
            edge_counts_.push_back(edge_number_);
          }
          sensor_msgs::PointCloud2 dbg_msg;
          pcl::toROSMsg(edge_cloud_lists[a], dbg_msg);
          dbg_msg.header.frame_id = "camera_init";
          edge_pub_.publish(dbg_msg);
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

  nh.param<float>("voxel_size", voxel_size_, 1);

  ros::Subscriber sub = nh.subscribe("point_cloud", 1, lidarCallback);

  plane_pub_       = nh.advertise<sensor_msgs::PointCloud2>("/voxel_plane", 100);
  edge_pub_        = nh.advertise<sensor_msgs::PointCloud2>("/lidar_edge", 100);
  color_plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/color_cloud", 100);

  ros::spin();
  return 0;
}
