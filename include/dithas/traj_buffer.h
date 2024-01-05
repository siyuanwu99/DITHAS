/**
 * @file traj_buffer.h
 * @brief
 * 1. 该轨迹类需要任意分配内存，因为轨迹可能很长 (trivial)
 * 2. 发布轨迹上的所有点给其他飞机
 * 3. 其他飞机收到整条轨迹，优化后更新轨迹上的所有点 （不考虑时间）
 * 4. 真的需要遍历
 * 5. Support any other trajectory class
 * @date 2024-01-04
 * @version 1.0.0
 * @Copyright 2024 <siyuanwu99@gmail.com>
 */

#ifndef DITHAS_TRAJ_BUFFER_H_
#define DITHAS_TRAJ_BUFFER_H_

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>

#endif  // DITHAS_TRAJ_BUFFER_H_
