#ifndef SENTRY_CHASSIS_CONTROLLER_INVERSE_KINEMATICS_H
#define SENTRY_CHASSIS_CONTROLLER_INVERSE_KINEMATICS_H

#include <array>

namespace sentry_kinematics {

struct IKResult {
  std::array<double,4> wheel_angular_vel; // rad/s
  std::array<double,4> steer_angle;       // rad
  std::array<double,4> wheel_linear_vel;  // m/s
};

IKResult inverseKinematics(double vx, double vy, double wz,
                           double wheel_base, double wheel_track, double wheel_radius);

} // namespace sentry_kinematics

#endif // SENTRY_CHASSIS_CONTROLLER_INVERSE_KINEMATICS_H
