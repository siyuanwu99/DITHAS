/**
 * @file
 * @brief Various utility functions
 * @date 27/12/2023
 * @version 1.0.0
 * @CopyRight 2023 <siyuanwu99@gmail.com>
 */

#ifndef DITHAS_UTILS_H_
#define DITHAS_UTILS_H_
#include <ros/ros.h>
#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <string>
#include <utility>

namespace dithas {

/**
 * @brief Read the parameters from the parameter server
 *
 * @param param_name string
 * @return <true, data> if the parameter is loaded successfully
 * @return <false, data> if the parameter is not loaded successfully
 */
std::pair<bool, Eigen::Isometry3d> readTransform(const std::string &param_name);

/**
 * @brief print 4x4 transform matrix
 *
 * @param tf Eigen::Isometry3d transform
 */
void printTransform(const Eigen::Isometry3d &tf);

/**
 * @brief Generate a random RGB color
 *
 * @param color std::array<unsigned int, 3>  [Red, Green, Blue]]
 */
typedef std::array<unsigned int, 3> RGBColor;

void generateRandomRGBColor(RGBColor &color);

}  // namespace dithas

#endif  // DITHAS_UTILS_H_
