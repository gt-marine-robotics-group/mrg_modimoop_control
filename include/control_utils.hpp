#pragma once

#include <algorithm>
#include <cmath>

namespace sailbot_control
{

inline double wrapToPi(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(x, hi));
}

inline double rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
}

inline double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

}  // namespace sailbot_control