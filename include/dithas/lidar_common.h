/**
 * @file lidar_common.h
 * @brief
 *
 *  This file contains common definitions for lidar related functions.
 *
 * @date 2024-01-11
 * @version 1.0.0
 * @Copyright 2024 <siyuanwu99@gmail.com>
 */

#ifndef LIDAR_COMMON_H_
#define LIDAR_COMMON_H_
#include <pcl/common/common.h>

class VOXEL_LOC {
 public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

typedef struct Voxel {
  float                                size;
  Eigen::Vector3d                      voxel_origin;
  std::vector<uint8_t>                 voxel_color;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  Voxel(float _size) : size(_size) {
    voxel_origin << 0, 0, 0;
    cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  }
} Voxel;

typedef struct SinglePlane {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZ                   p_center;
  Eigen::Vector3d                 normal;
  int                             index;
} SinglePlane;

/**
 * @brief solve a system of linear equations using Cramer's Rule
 *
 * @tparam T type of the matrix
 * @param matrix [TODO:parameter]
 * @param solution [TODO:parameter]
 */
template <class T>
void calc(T matrix[4][5], Eigen::Vector3d& solution) {
  // 1. claculate the determinate
  T base_D = matrix[1][1] * matrix[2][2] * matrix[3][3] +
             matrix[2][1] * matrix[3][2] * matrix[1][3] +
             matrix[3][1] * matrix[1][2] * matrix[2][3];
  base_D = base_D - (matrix[1][3] * matrix[2][2] * matrix[3][1] +
                     matrix[1][1] * matrix[2][3] * matrix[3][2] +
                     matrix[1][2] * matrix[2][1] * matrix[3][3]);

  if (base_D != 0) {
    T x_D = matrix[1][4] * matrix[2][2] * matrix[3][3] +
            matrix[2][4] * matrix[3][2] * matrix[1][3] + matrix[3][4] * matrix[1][2] * matrix[2][3];
    x_D = x_D -
          (matrix[1][3] * matrix[2][2] * matrix[3][4] + matrix[1][4] * matrix[2][3] * matrix[3][2] +
           matrix[1][2] * matrix[2][4] * matrix[3][3]);
    T y_D = matrix[1][1] * matrix[2][4] * matrix[3][3] +
            matrix[2][1] * matrix[3][4] * matrix[1][3] + matrix[3][1] * matrix[1][4] * matrix[2][3];
    y_D = y_D -
          (matrix[1][3] * matrix[2][4] * matrix[3][1] + matrix[1][1] * matrix[2][3] * matrix[3][4] +
           matrix[1][4] * matrix[2][1] * matrix[3][3]);
    T z_D = matrix[1][1] * matrix[2][2] * matrix[3][4] +
            matrix[2][1] * matrix[3][2] * matrix[1][4] + matrix[3][1] * matrix[1][2] * matrix[2][4];
    z_D = z_D -
          (matrix[1][4] * matrix[2][2] * matrix[3][1] + matrix[1][1] * matrix[2][4] * matrix[3][2] +
           matrix[1][2] * matrix[2][1] * matrix[3][4]);

    T x = x_D / base_D;
    T y = y_D / base_D;
    T z = z_D / base_D;
    // cout << "[ x:" << x << "; y:" << y << "; z:" << z << " ]" << endl;
    solution[0] = x;
    solution[1] = y;
    solution[2] = z;
  } else {
    std::cout << "【无解】";
    solution[0] = 0;
    solution[1] = 0;
    solution[2] = 0;
    //        return DBL_MIN;
  }
}

#endif  // LIDAR_COMMON_H_
