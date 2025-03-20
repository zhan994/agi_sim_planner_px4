/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <queue>

#include "ros_callback.h"
#include <Eigen/Dense>

struct Desired_State_t {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  Eigen::Quaterniond q;
  double yaw;
  double yaw_rate;
  double head_rate;

  Desired_State_t(){};

  Desired_State_t(Odom_Data_t &odom)
      : p(odom.p), v(Eigen::Vector3d::Zero()), a(Eigen::Vector3d::Zero()),
        j(Eigen::Vector3d::Zero()), q(odom.q),
        yaw(uav_utils::get_yaw_from_quaternion(odom.q)), yaw_rate(0),
        head_rate(0){};
};

struct Controller_Output_t {

  // Orientation of the body frame with respect to the world frame
  Eigen::Quaterniond q;
  Eigen::Quaterniond orientation;

  // Body rates in body frame
  Eigen::Vector3d bodyrates; // [rad/s]
  double roll_rate;
  double pitch_rate;
  double yaw_rate;
  double normalized_thrust;
  // Collective mass normalized thrust
  double thrust;

  Eigen::Vector3d des_v_real;
};

struct SO3_Controller_Output_t {
  Eigen::Matrix3d Rdes;
  Eigen::Vector3d Fdes;
  double net_force;
};

class LinearControl {
public:
  LinearControl(Parameter_t &);

  Eigen::Vector3d int_e_v;
  Eigen::Matrix3d Kp;
  Eigen::Matrix3d Kv;
  Eigen::Matrix3d Kvi;
  Eigen::Matrix3d Ka;
  double Kyaw;

  quadrotor_msgs::Px4ctrlDebug update(const Desired_State_t &des,
                                      const Odom_Data_t &odom,
                                      const Imu_Data_t &imu,
                                      Controller_Output_t &u,
                                      SO3_Controller_Output_t &u_so3);

  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
                                                const Odom_Data_t &odom,
                                                const Imu_Data_t &imu,
                                                Controller_Output_t &u);

  quadrotor_msgs::Px4ctrlDebug update_alg1(const Desired_State_t &des,
                                           const Odom_Data_t &odom,
                                           const Imu_Data_t &imu,
                                           Controller_Output_t &u);

  Controller_Output_t
  computeNominalReferenceInputs(const Desired_State_t &reference_state,
                                const Odom_Data_t &attitude_estimate) const;
  Eigen::Vector3d
  computeRobustBodyXAxis(const Eigen::Vector3d &x_B_prototype,
                         const Eigen::Vector3d &x_C, const Eigen::Vector3d &y_C,
                         const Eigen::Quaterniond &attitude_estimate) const;

  Eigen::Quaterniond
  computeDesiredAttitude(const Eigen::Vector3d &desired_acceleration,
                         const double reference_heading,
                         const Eigen::Quaterniond &attitude_estimate) const;

  bool estimateThrustModel(const Eigen::Vector3d &est_v);
  bool almostZero(const double value) const;
  bool almostZeroThrust(const double thrust_value) const;
  void resetThrustMapping(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;
  static constexpr double kAlmostZeroValueThreshold_ = 0.001;
  static constexpr double kAlmostZeroThrustThreshold_ = 0.01;

  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  void normalizeWithGrad(const Eigen::Vector3d &x, const Eigen::Vector3d &xd,
                         Eigen::Vector3d &xNor, Eigen::Vector3d &xNord) const;
  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
  double fromQuaternion2yaw(Eigen::Quaterniond q);
};

#endif