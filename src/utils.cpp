/**
 * @file
 * @brief This file contains the implementation of the functions defined in utils.h
 * @date 27/12/2023
 * @version 1.0.0
 * @CopyRight 2023 <siyuanwu99@gmail.com>
 * @see utils.h
 */

#include <dithas/utils.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace dithas {

namespace utils {

/**
 * @brief Read the parameters from the parameter server
 *
 * @param param_data std::vector<double>
 * @param param_name string
 * @return true if the parameter is loaded successfully
 */
std::pair<bool, Eigen::Affine3d> readTransform(const std::string &param_name) {
  std::string full_param_name = ros::this_node::getName() + "/" + param_name;
  ROS_INFO("Loading %s", full_param_name.c_str());
  Eigen::Affine3d     tf      = Eigen::Affine3d::Identity();
  bool                success = false;
  std::vector<double> param_data;
  if (ros::param::get(full_param_name, param_data)) {
    if (param_data.size() == 16) {
      ROS_INFO("Loaded %s successfully.", param_name.c_str());
      tf.matrix() << param_data[0], param_data[1], param_data[2], param_data[3], param_data[4],
          param_data[5], param_data[6], param_data[7], param_data[8], param_data[9], param_data[10],
          param_data[11], param_data[12], param_data[13], param_data[14], param_data[15];
      success = true;
    } else {
      ROS_ERROR("The size of the data is not 4x4.");
    }
  } else {
    ROS_ERROR("Failed to load %s.", param_name.c_str());
  }

  return std::make_pair(success, tf);
}

/**
 * @brief print 4x4 transform matrix
 *
 * @param tf [TODO:parameter]
 */
void printTransform(const Eigen::Affine3d &tf) {
  Eigen::Matrix4d   mat = tf.matrix();
  std::stringstream ss;

  ss << "world_T_marker:" << std::endl;
  ss << "  rows: 4" << std::endl;
  ss << "  cols: 4" << std::endl;
  ss << "  dt: d" << std::endl;
  ss << "  data: [";

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      ss << mat(i, j);

      if (i != mat.rows() - 1 || j != mat.cols() - 1) {
        ss << ", ";
      }
    }
  }

  ss << "]" << std::endl;

  std::cout << ss.str() << std::endl;
  std::cout << "-------------------------" << std::endl;
}

}  // namespace utils

}  // namespace dithas
