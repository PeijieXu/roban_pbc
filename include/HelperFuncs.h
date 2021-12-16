#pragma once

#include <RobotParams.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include <ros/ros.h>

#define PI 3.14159265359
#define RadToAngle 57.296

/**
 * @brief convert joint's angle to value
 *
 * @param dxlID joint ID
 * @param motoAngle angle of joint
 * @return integer digital value
 */
inline int convertAngleTOValue(uint8_t dxlID, double motoAngle) {
  return (motoAngle * AngleAlpha[dxlID - 1] + 2048);
}  // angle_to_value

// function for debug
void sysPause(int s = 0) {
  std::cout << s << std::endl;
  getchar();
}

void printVector(std::vector<double>& v) {
  auto it = v.begin();
  for (; it != v.begin() + 6; it++)
    (*it < 0) ? (std::cout << *it << " ") : (std::cout << " " << *it << " ");
  std::cout << std::endl;
  for (; it != v.begin() + 12; it++)
    (*it < 0) ? (std::cout << *it << " ") : (std::cout << " " << *it << " ");
  std::cout << std::endl;
  for (; it != v.begin() + 15; it++)
    (*it < 0) ? (std::cout << *it << " ") : (std::cout << " " << *it << " ");
  std::cout << std::endl;
  for (; it != v.end(); it++)
    (*it < 0) ? (std::cout << *it << " ") : (std::cout << " " << *it << " ");
  std::cout << std::endl << std::endl;
}

void printPos(const Eigen::Vector3d& v) {
  std::cout << v.transpose() << std::endl;
}

inline void debug(double t, std::string name = "") {
  std::string prt = "DEBUG " + name + " %.4f";
  ROS_INFO(prt.c_str(), t);
}

template <typename E>
inline E Max(E a, E b) {
  return (a > b) ? a : b;
}

template <typename E>
inline E Min(E a, E b) {
  return (a < b) ? a : b;
}
