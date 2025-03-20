#include "controller1.h"
#include "std_msgs/Float32.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <uav_utils/converters.h>

using namespace std;
using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

LinearControl::LinearControl(Parameter_t &param) : param_(param) {

  resetThrustMapping();
}

/*
  Fast_250 low_level_controller
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/

quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
                                const Odom_Data_t &odom, const Imu_Data_t &imu,
                                Controller_Output_t &u) {

  // compute disired acceleration
  Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
  Eigen::Vector3d Kp, Kv;
  Kp << param_.normal_gain.Kp0, param_.normal_gain.Kp1, param_.normal_gain.Kp2;
  Kv << param_.normal_gain.Kv0, param_.normal_gain.Kv1, param_.normal_gain.Kv2;
  des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) +
            Kp.asDiagonal() * (des.p - odom.p);
  des_acc += Eigen::Vector3d(0, 0, param_.gra);

  u.thrust = computeDesiredCollectiveThrustSignal(des_acc);

  //计算角度输出给姿态控制器
  double roll, pitch, yaw, yaw_imu;
  double yaw_odom = fromQuaternion2yaw(odom.q);
  double sin = std::sin(yaw_odom);
  double cos = std::cos(yaw_odom);
  roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
  pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
  // yaw = fromQuaternion2yaw(des.q);
  //根据四元数计算出欧拉角，ros的odom消息机制为ZYX，无人机的是ZXY
  yaw_imu = fromQuaternion2yaw(imu.q);
  // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
  //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
  //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
  Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  u.q = imu.q * odom.q.inverse() * q; // Align with FCU frame

  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);

  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);

  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();

  debug_msg_.des_thr = u.thrust;

  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100) {

    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage
*/
double LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc) {

  double throttle_percentage(0.0);

  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool LinearControl::estimateThrustModel(const Eigen::Vector3d &est_a) {

  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1) {

    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) { // 45ms

      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) { // 35ms

      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();

    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    if (param_.thr_map.print_val)
      printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    // fflush(stdout);

    debug_msg_.hover_percentage = thr2acc_;
    return true;
  }
  return false;
}

void LinearControl::resetThrustMapping(void) {

  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}

void LinearControl::normalizeWithGrad(const Eigen::Vector3d &x,
                                      const Eigen::Vector3d &xd,
                                      Eigen::Vector3d &xNor,
                                      Eigen::Vector3d &xNord) const {

  const double xSqrNorm = x.squaredNorm();
  const double xNorm = sqrt(xSqrNorm);
  xNor = x / xNorm;
  xNord = (xd - x * (x.dot(xd) / xSqrNorm)) / xNorm;
  return;
}

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q) {

  double yaw =
      atan2(2 * (q.x() * q.y() + q.w() * q.z()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return yaw;
}