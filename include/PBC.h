/**
 * @file PBC.h
 * @author Peijie Xu (peijiexu99@gmail.com)
 * @brief Position Based Controller (PBC) Header
 * @version 1.0
 * @date 2021-12-16
 *
 * @copyright  Peijie Xu 2021
 *
 *
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to
 *  deal in the Software without restriction, including without limitation the
 *  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 *  sell copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:

 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.

 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 *  IN THE SOFTWARE.
 *
 *  WARNING:
 *  If any person/robot use this robot control code means he/she/it will take
 *  all responsibilities for any damage.
 *
 *  All robots who use this robot control code MUST obey The three laws of
 *  Robotics: 1) A robot may not injure a human being or, through inaction,
 *  allow a human being to come to harm 2) A robot must obey orders givein to it
 *  by human beings except where such orders would conflict with the First Law.
 *  3) A robot must protect its own existence as long as such protection does
 *  not conflict with the First or Second Law.
 *
 */

#pragma once

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "bodyhub/JointControlPoint.h"
#include "bodyhub/SrvServoAllRead.h"
// #include "dynamixel_sdk/dynamixel_sdk.h"
// #include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
}

class PBC {
 public:
  // Node Handle in ROS
  ros::NodeHandle nh;

  /**
   * @brief Constructor: create a new Position Based Controller (PBC) object
   * @param n NodeHandle in ROS
   * @param F Frequence publish thread
   */
  PBC(ros::NodeHandle n, int F);

  /**
   * @brief Destroy the PBC object
   */
  ~PBC() {}

  /**
   * @brief Initialize the position of center of mass (COM), left foot, and
   *        right foot
   */
  void poseInit();

  /**
   * @brief In real world, reset the robot's pose to pre-walking status
   *
   */
  void resetPose();

  /**
   * @brief In simulation environment, reset the robot's pose to
   *        pre-walking status
   *
   */
  void simReset();

  /**
   * @brief Do inverse kinematic, push the target joint value to the publish
   *        thread.
   *
   */
  void updateAngleOfJoint();

  /**
   * @brief Do inverse kinematic of 2 legs, then push the target joint value to
   *        the publish thread
   *
   * @param comPos position of COM
   * @param lFootPos position of left foot
   * @param rFootPos position of right foot
   */
  void updateAngleOfJoint(Eigen::Vector3d const& comPos,
                          Eigen::Vector3d const& lFootPos,
                          Eigen::Vector3d const& rFootPos);

  /**
   * @brief Get the Angle Of Joints
   *
   * @param isPrint true if you want to print the real time angle
   * @return true - when everything goes right; false - when fail to request for
   *         the joints' angle
   */
  bool getAngleOfJoint(bool isPrint = false);

  /**
   * @brief move the COM to the target position in X dimension
   *
   * @param comPosEnd the target position
   */
  void moveCOM(const double comPosEnd);

  /**
   * @brief Take the first step: move left the robot's COM, lift its right leg,
   *        then move the COM forward
   *
   */
  void firstStep();

  /**
   * @brief normal step
   *
   * @param moveRightLeg true if the robot should lift its right leg
   */
  void step(bool moveRightLeg);

  /**
   * @brief last step to take
   *
   * @param moveRightLeg true if the robot should lift its right leg
   */
  void lastStep(bool moveRightLeg);

  /**
   * @brief waiting for PubQue to be empty, so the robot finish all movements
   *
   * @return true if no target position needs to be published
   *
   */
  bool waitPubQueEmpty();

 private:
  double footDis;  // the distance between the COM and one foot in Y dimension
  double comH;     // the height of COM

  int pubThreadFreq;  // desired published frequency

  std::mutex mtx;  // mutex for jVVToSendQueue

  std::vector<double> jointValueVectorToSend;
  std::vector<double> jointValueVectorNow;
  std::deque<std::vector<double>> jVVToSendQueue;

  // position of 3 key points of the robot.
  // comPos -- position of COM
  // lFootPos -- position of left foot
  // rFootPos -- position of right foot
  Eigen::Vector3d comPos, lFootPos, rFootPos;

  // the output vector for GenerateCuicSpline()
  Eigen::Vector4d comOutX, comOutY, comOutZ;
  Eigen::Vector4d lFootOutX, lFootOutY, lFootOutZ;
  Eigen::Vector4d rFootOutX, rFootOutY, rFootOutZ;

  // client of srvice "MediumSize/BodyHub/GetJointAngle" to get angle of joints
  ros::ServiceClient bodyhubJointAngleClient;

  /**
   * @brief plan the trajectory of a point (COM/ left foot/ right foot)
   *
   * @param p array of coordinates (size = 4) that the point must arrive
   * @param t array of time sequence (size = 4)
   * @param splineX output vector, stores the coefficient of the polynomial
   *                which symbolizes the trajectory of X dimension
   * @param splineY output vector, stores the coefficient of the polynomial
   *                which symbolizes the trajectory of Y dimension
   * @param splineZ output vector, stores the coefficient of the polynomial
   *                which symbolizes the trajectory of Z dimension
   */
  void generateCuicSpline3D(const Eigen::Vector3d p[], const double t[],
                            Eigen::Vector4d& splineX, Eigen::Vector4d& splineY,
                            Eigen::Vector4d& splineZ);

  /**
   * @brief plan the trajectory of one dimension (X/ Y/ Z) of a point
   *
   * @return Eigen::Vector4d -- output vector, stores the coefficient of the
   *         polynomial which symbolizes the trajectory of one dimension
   */
  Eigen::Vector4d generateCuicSpline1D(const double[], const double[]);

  /**
   * @brief Calculate the coordinate of a point
   *
   * @param splineX input vector, stores the coefficient of the polynomial
   *                which symbolizes the trajectory of X dimension
   * @param splineY input vector, stores the coefficient of the polynomial
   *                which symbolizes the trajectory of Y dimension
   * @param splineZ input vector, stores the coefficient of the polynomial
   *                which symbolizes the trajectory of Z dimension
   * @param t time
   * @return Eigen::Vector3d -- the coordinate of the point at time t
   */
  Eigen::Vector3d cubicSpline3D(const Eigen::Vector4d& splineX,
                                const Eigen::Vector4d& splineY,
                                const Eigen::Vector4d& splineZ, double t);
  /**
   * @brief Publish thread to send target joints' angle
   *
   */
  void PubThread();

  /**
   * @brief Unused publish thread, require bottom layer coding
   *
   */
  void PubThread1();

  /**
   * @brief Do inverse kinematic of one leg
   *
   * @param comPos position of COM
   * @param footPos position of foot
   * @param isRightLeg true if the target leg is the right one
   * @return true -- when successfully done the inverse kinematic
   */
  bool IKLeg(Eigen::Vector3d const& comPos, Eigen::Vector3d const& footPos,
             bool isRightLeg);
};
