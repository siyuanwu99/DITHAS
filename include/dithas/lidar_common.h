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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define HASH_P            116101
#define MAX_N             10000000019
#define SMALL_EPS         1e-10
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0
#define PLM(a) \
  vector<Eigen::Matrix<double, a, a>, Eigen::aligned_allocator<Eigen::Matrix<double, a, a>>>
#define PLV(a) \
  vector<Eigen::Matrix<double, a, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, a, 1>>>
#define VEC(a) Eigen::Matrix<double, a, 1>

#define G_m_s2 9.81
#define DIMU   18
#define DIM    15
#define DNOI   12
#define NMATCH 5
#define DVEL   6

typedef pcl::PointXYZI PointType;

/**
 * @class VOXEL_LOC
 * @brief location of a voxel in the voxel grid
 */
class VOXEL_LOC {
 public:
  int64_t x, y, z;

  explicit VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

namespace std {
template <>
struct hash<VOXEL_LOC> {
  size_t operator()(const VOXEL_LOC& s) const {
    using std::hash;
    using std::size_t;
    // return (((hash<int64_t>()(s.z)*HASH_P)%MAX_N + hash<int64_t>()(s.y))*HASH_P)%MAX_N +
    // hash<int64_t>()(s.x);
    int64_t index_x, index_y, index_z;
    double  cub_len = 0.125;
    index_x         = static_cast<int>(round(floor((s.x) / cub_len + SMALL_EPS)));
    index_y         = static_cast<int>(round(floor((s.y) / cub_len + SMALL_EPS)));
    index_z         = static_cast<int>(round(floor((s.z) / cub_len + SMALL_EPS)));
    return (((((index_z * HASH_P) % MAX_N + index_y) * HASH_P) % MAX_N) + index_x) % MAX_N;
  }
};
}  // namespace std

/**
 * @class Voxel
 * @brief voxel which contains a point cloud
 */
typedef struct Voxel {
  float                                size;          // voxel size
  Eigen::Vector3d                      voxel_origin;  // origin point of the voxel
  std::vector<uint8_t>                 voxel_color;   // visualization color of the voxel
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;         // point cloud in the voxel
  Voxel(float _size) : size(_size) {
    voxel_origin << 0, 0, 0;
    cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  }
} Voxel;

struct M_POINT {
  float xyz[3];
  int   count = 0;
};

/**
 * @class SinglePlane
 * @brief a plane with a point cloud
 */
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
  // calculate the determinate
  T base_D = matrix[1][1] * matrix[2][2] * matrix[3][3] +
             matrix[2][1] * matrix[3][2] * matrix[1][3] +
             matrix[3][1] * matrix[1][2] * matrix[2][3];
  base_D = base_D - (matrix[1][3] * matrix[2][2] * matrix[3][1] +
                     matrix[1][1] * matrix[2][3] * matrix[3][2] +
                     matrix[1][2] * matrix[2][1] * matrix[3][3]);

  if (base_D != 0) {  // if matrix is non-singular
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
    std::cout << "[ERROR][Cramer Solver] Matrix is singular!" << std::endl;
    solution[0] = 0;
    solution[1] = 0;
    solution[2] = 0;
  }
}

/**
 * @brief downsample the point cloud with voxel size
 *
 * @param pc point cloud input
 * @param voxel_size voxel size
 */
void downsampleVoxel(pcl::PointCloud<PointType>& pc, double voxel_size);

#endif  // LIDAR_COMMON_H_
