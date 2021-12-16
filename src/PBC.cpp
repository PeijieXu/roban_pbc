/**
 * @file PBC.cpp
 * @author Peijie Xu (peijiexu99@gmail.com)
 * @brief Position Based Controller (PBC)
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

#include <HelperFuncs.h>
#include <PBC.h>
#include <RobotParams.h>

#include <chrono>
#include <cstdio>

#define SERVO_ERR 0.003

bool PBC::getAngleOfJoint(bool isPrint) {
  bodyhub::SrvServoAllRead getJointAngle;

  if (!bodyhubJointAngleClient.call(getJointAngle)) {
    ROS_ERROR("PBC: request for joint angle failed!");
    return false;
  }

  jointValueVectorNow.assign(getJointAngle.response.getData.begin(),
                             getJointAngle.response.getData.end());

  if (isPrint) printVector(jointValueVectorNow);

  return true;
}

void PBC::updateAngleOfJoint() {
  updateAngleOfJoint(comPos, lFootPos, rFootPos);
}

PBC::PBC(ros::NodeHandle n, int F) : nh(n), pubThreadFreq(F) {
  jointValueVectorToSend.reserve(JOINT_NUM);
  jointValueVectorNow.reserve(JOINT_NUM);

  bodyhubJointAngleClient = nh.serviceClient<bodyhub::SrvServoAllRead>(
      "MediumSize/BodyHub/GetJointAngle");

  if (getAngleOfJoint())
    jointValueVectorToSend.assign(jointValueVectorNow.begin(),
                                  jointValueVectorNow.end());

  // start the publish thread
  std::thread(std::bind(&PBC::PubThread, this)).detach();

  poseInit();
}

void PBC::poseInit() {
  footDis = L_bw + 0.01;
  comH = L_lh + L_bh;

  comPos = Eigen::Vector3d(0, 0.00, comH - 0.02);  // min: comH-0.07
  lFootPos = Eigen::Vector3d(0.0, footDis, 0);
  rFootPos = Eigen::Vector3d(0.0, -footDis, 0);
  // X max < 0.08(comH-0.02) 0.11（comH-0.04)
}

// robot move functions

void PBC::moveCOM(const double comPosEnd) {
  const double comPosStart = comPos[1];

  const int direction = (comPosEnd > comPosStart) ? 1 : -1;
  const double comPosMid = (comPosStart + comPosEnd) * 0.7;

  if (direction == 1) {
    // printf("COM moving left\n");
    for (int i = 0; comPos[1] < comPosEnd; ++i) {
      comPos[1] +=
          (comPos[1] < comPosMid) ? (0.004 * direction) : (0.001 * direction);
      updateAngleOfJoint();
    }

  } else {
    // printf("COM moving right\n");

    for (int i = 0; comPos[1] > comPosEnd; ++i) {
      comPos[1] +=
          (comPos[1] > comPosMid) ? (0.004 * direction) : (0.001 * direction);
      updateAngleOfJoint();
    }
  }
}

void PBC::firstStep() {
  const double comPosEnd = 0.06;

  double t[] = {0, 1.6, 3.2, 4.4};
  for (auto& time : t) time *= pubThreadFreq;

  Eigen::Vector3d comPosTra[] = {Eigen::Vector3d(0.00, comPosEnd, comH - 0.02),
                                 Eigen::Vector3d(0.00, 0.067, comH - 0.023),
                                 Eigen::Vector3d(0.00, 0.067, comH - 0.025),
                                 Eigen::Vector3d(0.07, 0.00, comH - 0.025)};
  Eigen::Vector3d lFootPosTra[] = {Eigen::Vector3d(0.00, footDis, 0.00),
                                   Eigen::Vector3d(0.00, footDis, 0.00),
                                   Eigen::Vector3d(0.00, footDis, 0.00),
                                   Eigen::Vector3d(0.00, footDis, 0.00)};
  Eigen::Vector3d rFootPosTra[] = {Eigen::Vector3d(0.00, -footDis, 0.00),
                                   Eigen::Vector3d(0.03, -footDis, 0.05),
                                   Eigen::Vector3d(0.062, -footDis, 0.03),
                                   Eigen::Vector3d(0.07, -footDis, 0.00)};
  // plan the trajectory
  generateCuicSpline3D(comPosTra, t, comOutX, comOutY, comOutZ);
  generateCuicSpline3D(lFootPosTra, t, lFootOutX, lFootOutY, lFootOutZ);
  generateCuicSpline3D(rFootPosTra, t, rFootOutX, rFootOutY, rFootOutZ);

  // move left the robot's COM
  moveCOM(comPosEnd);

  for (int i = 0; i <= t[3]; ++i) {
    comPos = cubicSpline3D(comOutX, comOutY, comOutZ, i);
    lFootPos = cubicSpline3D(lFootOutX, lFootOutY, lFootOutZ, i);
    rFootPos = cubicSpline3D(rFootOutX, rFootOutY, rFootOutZ, i);
    updateAngleOfJoint(comPos, lFootPos, rFootPos);
  }
}

void PBC::step(bool moveRightLeg) {
  auto& movingLeg = moveRightLeg ? rFootPos : lFootPos;
  auto& stillLeg = moveRightLeg ? lFootPos : rFootPos;
  const double offsetX = stillLeg[0];
  comPos[0] -= offsetX;
  lFootPos[0] -= offsetX;
  rFootPos[0] -= offsetX;

  const int direction = moveRightLeg ? 1 : -1;
  const double comPosEnd = 0.05 * direction;

  double t[] = {0, 1.6, 3.2, 4.4};
  for (auto& time : t) time *= pubThreadFreq;

  Eigen::Vector3d comPosTra[] = {
      Eigen::Vector3d(0.00, comPosEnd, comH - 0.025),
      Eigen::Vector3d(0.001, 0.07 * direction, comH - 0.025),
      Eigen::Vector3d(0.002, 0.067 * direction, comH - 0.025),
      Eigen::Vector3d(0.08, 0.00 * direction, comH - 0.025)};

  Eigen::Vector3d stillLegPosTra[] = {
      Eigen::Vector3d(stillLeg), Eigen::Vector3d(stillLeg),
      Eigen::Vector3d(stillLeg), Eigen::Vector3d(stillLeg)};

  Eigen::Vector3d movingLegPosTra[] = {
      Eigen::Vector3d(movingLeg),
      Eigen::Vector3d(-0.002, -footDis * direction, 0.05),
      Eigen::Vector3d(0.02, -footDis * direction, 0.04),
      Eigen::Vector3d(0.08, -footDis * direction, 0.00)};
  //
  Eigen::Vector4d stillFootOutX, stillFootOutY, stillFootOutZ;
  Eigen::Vector4d movingFootOutX, movingFootOutY, movingFootOutZ;
  generateCuicSpline3D(comPosTra, t, comOutX, comOutY, comOutZ);
  generateCuicSpline3D(stillLegPosTra, t, stillFootOutX, stillFootOutY,
                       stillFootOutZ);
  generateCuicSpline3D(movingLegPosTra, t, movingFootOutX, movingFootOutY,
                       movingFootOutZ);

  moveCOM(comPosEnd);

  for (int i = 0; i <= t[3]; ++i) {
    // if (i % 5 == 0)
    // printf("%.3f ", comPos[0]);
    comPos = cubicSpline3D(comOutX, comOutY, comOutZ, i);
    stillLeg = cubicSpline3D(stillFootOutX, stillFootOutY, stillFootOutZ, i);
    movingLeg =
        cubicSpline3D(movingFootOutX, movingFootOutY, movingFootOutZ, i);
    updateAngleOfJoint();
  }
}

void PBC::lastStep(bool moveRightLeg) {
  auto& movingLeg = moveRightLeg ? rFootPos : lFootPos;
  auto& stillLeg = moveRightLeg ? lFootPos : rFootPos;
  const double offsetX = stillLeg[0];
  comPos[0] -= offsetX;
  lFootPos[0] -= offsetX;
  rFootPos[0] -= offsetX;

  const int direction = moveRightLeg ? 1 : -1;
  const double comPosEnd = 0.05 * direction;

  double t[] = {0, 0.8, 1.6, 2.2};
  for (auto& time : t) time *= pubThreadFreq;

  Eigen::Vector3d comPosTra[] = {
      Eigen::Vector3d(0.00, comPosEnd, comH - 0.025),
      Eigen::Vector3d(0.00, 0.07 * direction, comH - 0.025),
      Eigen::Vector3d(0.00, 0.067 * direction, comH - 0.025),
      Eigen::Vector3d(0.00, 0.00 * direction, comH - 0.025)};

  Eigen::Vector3d stillLegPosTra[] = {
      Eigen::Vector3d(stillLeg), Eigen::Vector3d(stillLeg),
      Eigen::Vector3d(stillLeg), Eigen::Vector3d(stillLeg)};

  Eigen::Vector3d movingLegPosTra[] = {
      Eigen::Vector3d(movingLeg),
      Eigen::Vector3d(-0.04, -footDis * direction, 0.03),
      Eigen::Vector3d(-0.01, -footDis * direction, 0.01),
      Eigen::Vector3d(-0.00, -footDis * direction, 0.00)};
  //
  Eigen::Vector4d stillFootOutX, stillFootOutY, stillFootOutZ;
  Eigen::Vector4d movingFootOutX, movingFootOutY, movingFootOutZ;
  generateCuicSpline3D(comPosTra, t, comOutX, comOutY, comOutZ);
  generateCuicSpline3D(stillLegPosTra, t, stillFootOutX, stillFootOutY,
                       stillFootOutZ);
  generateCuicSpline3D(movingLegPosTra, t, movingFootOutX, movingFootOutY,
                       movingFootOutZ);

  moveCOM(comPosEnd);

  for (int i = 0; i <= t[3]; ++i) {
    // if (i % 5 == 0)
    // printf("%.3f ", comPos[0]);
    comPos = cubicSpline3D(comOutX, comOutY, comOutZ, i);
    stillLeg = cubicSpline3D(stillFootOutX, stillFootOutY, stillFootOutZ, i);
    movingLeg =
        cubicSpline3D(movingFootOutX, movingFootOutY, movingFootOutZ, i);
    updateAngleOfJoint();
  }
}

void PBC::updateAngleOfJoint(Eigen::Vector3d const& comPos,
                             Eigen::Vector3d const& lFootPos,
                             Eigen::Vector3d const& rFootPos) {
  if (!IKLeg(comPos, lFootPos, false)) return;
  if (!IKLeg(comPos, rFootPos, true)) return;

  // push the target joint value to the publish thread
  mtx.lock();
  // if there are too many target joint value still waiting in the line, halve
  // them
  if (jVVToSendQueue.size() > 1000) {
    int i = 0;
    for (; 2 * i < jVVToSendQueue.size() && i < 1000; ++i)
      jVVToSendQueue.at(i) = jVVToSendQueue.at(2 * i);
    jVVToSendQueue.resize(i);
    ROS_INFO("halve jVVToSendQueue to %d", jVVToSendQueue.size());
  }

  jVVToSendQueue.emplace_back(jointValueVectorToSend);
  mtx.unlock();
}

bool PBC::IKLeg(Eigen::Vector3d const& comPos, Eigen::Vector3d const& footPos,
                bool isRightLeg) {
  static double x, y, z, z_2, ratio = L_lh / (L_lh + L_bh);
  double theta[6] = {0, 0, 0, 0, 0, 0};

  y = comPos[1] - footPos[1] + L_bw * (isRightLeg ? -1 : 1);
  z = comPos[2] - footPos[2];

  if (SERVO_ERR >= z) {
    ROS_INFO("IK: Leg Singular, foot Pos > com Pos");
    return false;
  }
  theta[1] = atan(y / z);

  x = footPos[0] - comPos[0];

  z_2 = (z * z + y * y) * ratio * ratio;
  z = sqrt(z_2);

  if (x > 0.2 || x < -0.2) {
    ROS_INFO("IK: Leg Singular, x = %.4f > max step length (L_th = 0.1)", x);
    return false;
  } else if (-SERVO_ERR + 0.002 < x && x < SERVO_ERR - 0.002) {
    if (0.157 >= z || z >= L_lh - SERVO_ERR) {
      ROS_INFO("IK: Leg Singular, z = %.4f", z);
      return false;
    } else {
      theta[2] = acos((L_th * L_th + z_2 - L_sh * L_sh) / (2 * L_th * z));
      double temp = (L_th * L_th - z_2 + L_sh * L_sh) / (2 * L_th * L_sh);
      if (temp > 1 || temp < -1) {
        ROS_INFO("IK: Leg Singular, z = %.4f", z);
        return false;
      }
      theta[3] = acos(temp) - PI;
    }

  } else {
    double temp = (z_2 + x * x - L_th * L_th - L_sh * L_sh) / (2 * L_th * L_sh);
    if (temp > 1 || temp < -1) {
      ROS_INFO("IK: Leg Singular, x= %.4f z = %.4f", x, z);
      return false;
    }
    theta[3] = -acos(temp);
    theta[2] = asin(sin(-theta[3]) * L_sh / sqrt(x * x + z_2)) + atan(x / z);
  }

  theta[1] *= RadToAngle;
  theta[2] *= RadToAngle * (isRightLeg ? -1 : 1);
  theta[3] *= RadToAngle * (isRightLeg ? -1 : 1);
  theta[4] = (footPos[2] < 0.1) ? (theta[2] + theta[3]) : 0;
  theta[5] = theta[1];

  const int indexOffset = (isRightLeg ? 6 : 0);
  for (int i = 0; i < 6; ++i)
    jointValueVectorToSend.at(i + indexOffset) = theta[i];
  return true;
}

void PBC::resetPose() {
  auto comPosStart = comPos;
  auto lFootPosStart = lFootPos;
  auto rFootPosStart = rFootPos;

  auto comPosEnd = Eigen::Vector3d(0, 0.00, comH - 0.02);  // min: comH-0.07
  auto lFootPosEnd = Eigen::Vector3d(0.0, footDis, 0);
  auto rFootPosEnd = Eigen::Vector3d(0.0, -footDis, 0);

  for (int i = 0; i < 50; ++i) {
    for (int j = 0; j < 3; ++j) {
      comPos[j] = (comPosEnd[j] - comPosStart[j]) / 50 * i + comPosStart[j];
      lFootPos[j] =
          (lFootPosEnd[j] - lFootPosStart[j]) / 50 * i + lFootPosStart[j];
      rFootPos[j] =
          (rFootPosEnd[j] - rFootPosStart[j]) / 50 * i + rFootPosStart[j];
    }

    updateAngleOfJoint();
  }
}

void PBC::simReset() {
  if (!IKLeg(comPos, lFootPos, false)) return;
  if (!IKLeg(comPos, rFootPos, true)) return;

  std::vector<double> diffOfjointValue(JOINT_NUM, 0);

  for (int i = 0; i < JOINT_NUM; ++i) {
    diffOfjointValue.at(i) = jointValueVectorToSend.at(i) / 20;
    jointValueVectorToSend.at(i) = 0;
  }

  mtx.lock();
  for (int j = 0; j < 20; ++j) {
    for (int i = 0; i < JOINT_NUM; ++i) {
      jointValueVectorToSend.at(i) += diffOfjointValue.at(i);
    }
    jVVToSendQueue.emplace_back(jointValueVectorToSend);
    // printVector(jointValueVectorToSend);
  }
  mtx.unlock();
}

void PBC::generateCuicSpline3D(const Eigen::Vector3d p[], const double t[],
                               Eigen::Vector4d& splineX,
                               Eigen::Vector4d& splineY,
                               Eigen::Vector4d& splineZ) {
  double x[] = {p[0][0], p[1][0], p[2][0], p[3][0]};
  double y[] = {p[0][1], p[1][1], p[2][1], p[3][1]};
  double z[] = {p[0][2], p[1][2], p[2][2], p[3][2]};
  splineX = generateCuicSpline1D(x, t);
  splineY = generateCuicSpline1D(y, t);
  splineZ = generateCuicSpline1D(z, t);
}

Eigen::Vector4d PBC::generateCuicSpline1D(const double in[], const double t[]) {
  Eigen::Matrix<double, 4, 4> mat4x4;
  mat4x4 << 1, t[0], pow(t[0], 2), pow(t[0], 3), 1, t[1], pow(t[1], 2),
      pow(t[1], 3), 1, t[2], pow(t[2], 2), pow(t[2], 3), 1, t[3], pow(t[3], 2),
      pow(t[3], 3);

  return mat4x4.inverse() * Eigen::Vector4d(in[0], in[1], in[2], in[3]);
}

Eigen::Vector3d PBC::cubicSpline3D(const Eigen::Vector4d& splineX,
                                   const Eigen::Vector4d& splineY,
                                   const Eigen::Vector4d& splineZ, double t) {
  return Eigen::Vector3d(splineX(0) + splineX(1) * t + splineX(2) * pow(t, 2) +
                             splineX(3) * pow(t, 3),
                         splineY(0) + splineY(1) * t + splineY(2) * pow(t, 2) +
                             splineY(3) * pow(t, 3),
                         splineZ(0) + splineZ(1) * t + splineZ(2) * pow(t, 2) +
                             splineZ(3) * pow(t, 3));
}

bool PBC::waitPubQueEmpty() {
  // printf("waiting for PubQue to be empty ");
  while (ros::ok()) {
    mtx.lock();

    if (jVVToSendQueue.size() <= 1) {
      // printf("\n");
      return true;
    }

    mtx.unlock();

    ros::Rate(2).sleep();
    // printf(". ");
  }
}

void PBC::PubThread() {
  // Msgs
  bodyhub::JointControlPoint jointValueMsg;

  ros::Publisher MotoPositionPub = nh.advertise<bodyhub::JointControlPoint>(
      "MediumSize/BodyHub/MotoPosition", 10);

  ros::Rate pub_rate(this->pubThreadFreq);
  while (ros::ok() && this) {
    mtx.lock();
    if (!jVVToSendQueue.empty()) {
      auto jVV = jVVToSendQueue.front();
      jointValueMsg.positions.assign(jVV.begin(), jVV.end());

      if (jVVToSendQueue.size() > 1) jVVToSendQueue.pop_front();
    }
    mtx.unlock();

    MotoPositionPub.publish(jointValueMsg);
    jointValueMsg.positions.clear();

    pub_rate.sleep();
  }
}

void PBC::PubThread1() {
  // // TODO 采用精准定时
  // using namespace std;
  // using namespace chrono;

  // ros::Rate pub_rate(this->pubThreadFreq);

  // DynamixelWorkbench dxl_wb;
  // int32_t goalPosition[JOINT_NUM];
  // uint8_t idArray[JOINT_NUM];
  // for (int i = 0; i < JOINT_NUM; ++i) idArray[i] = i + 1;
  // uint8_t idCnt = JOINT_NUM;
  // const char* log = NULL;

  // std::vector<double> jVV(JOINT_NUM, 0);

  // while (ros::ok()) {
  //   auto start = system_clock::now();

  //   mtx.lock();
  //   if (!jVVToSendQueue.empty()) {
  //     auto jVVToSend = jVVToSendQueue.front();
  //     jVV.assign(jVVToSend.begin(), jVVToSend.end());

  //     if (jVVToSendQueue.size() > 1) jVVToSendQueue.pop_front();

  //   } else {
  //     mtx.unlock();
  //     continue;
  //   }
  //   mtx.unlock();

  //   for (int idNum = 0; idNum < JOINT_NUM; idNum++) {
  //     goalPosition[idNum] =
  //         convertAngleTOValue(idNum + 1, jVV.at(idNum)) + servoOffset[idNum];
  //     std::cout << goalPosition[idNum] << " ";
  //   }
  //   std::cout << std::endl;

  //   bool res = dxl_wb.syncWrite(0, idArray, idCnt, goalPosition, 1, &log);
  //   if (!res) ROS_ERROR("SigHandler()---%s", log);

  //   pub_rate.sleep();

  //   auto end = system_clock::now();
  //   auto duration = duration_cast<microseconds>(end - start);

  //   cout << double(duration.count()) * microseconds::period::num << " us"
  //        << endl;
  // }
}
