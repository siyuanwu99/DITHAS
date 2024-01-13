/**
 * @file lidar_common.cpp
 * @brief
 *
 *  This file contains common definitions for lidar related functions.
 *
 * @date 2024-01-11
 * @version 1.0.0
 * @Copyright 2024 <siyuanwu99@gmail.com>
 */

#include <dithas/lidar_common.h>
#include <unordered_map>

void downsampleVoxel(pcl::PointCloud<PointType>& pc, double voxel_size) {
  if (voxel_size < 0.01) return;

  std::unordered_map<VOXEL_LOC, M_POINT> feature_map;
  size_t                                 pt_size = pc.size();

  for (size_t i = 0; i < pt_size; i++) {
    PointType& pt_trans = pc[i];
    float      loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pt_trans.data[j] / voxel_size;
      if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto      iter = feature_map.find(position);
    if (iter != feature_map.end()) {
      iter->second.xyz[0] += pt_trans.x;
      iter->second.xyz[1] += pt_trans.y;
      iter->second.xyz[2] += pt_trans.z;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0]            = pt_trans.x;
      anp.xyz[1]            = pt_trans.y;
      anp.xyz[2]            = pt_trans.z;
      anp.count             = 1;
      feature_map[position] = anp;
    }
  }

  pt_size = feature_map.size();
  pc.clear();
  pc.resize(pt_size);

  size_t i = 0;
  for (auto iter = feature_map.begin(); iter != feature_map.end(); ++iter) {
    pc[i].x = iter->second.xyz[0] / iter->second.count;
    pc[i].y = iter->second.xyz[1] / iter->second.count;
    pc[i].z = iter->second.xyz[2] / iter->second.count;
    i++;
  }
}
