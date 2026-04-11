#include "path_planner.hpp"

#include <cmath>
#include <limits>

namespace mrg_modimoop_planner
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEarthRadiusM = 6371000.0;
}  // namespace

Point2D PathPlanner::geo_to_local_en(
  const GeoPoint & origin,
  const GeoPoint & target)
{
  const double lat0 = origin.latitude_deg * kPi / 180.0;
  const double lat1 = target.latitude_deg * kPi / 180.0;
  const double dlat = lat1 - lat0;
  const double dlon = (target.longitude_deg - origin.longitude_deg) * kPi / 180.0;

  Point2D p;
  p.x_east_m = kEarthRadiusM * dlon * std::cos(0.5 * (lat0 + lat1));
  p.y_north_m = kEarthRadiusM * dlat;
  return p;
}

PathPlanner::PathPlanner(const PlannerParams & params)
: params_(params)
{
}

void PathPlanner::set_params(const PlannerParams & params)
{
  params_ = params;
}

const PlannerParams & PathPlanner::params() const
{
  return params_;
}

double PathPlanner::wrap_to_2pi(double angle)
{
  while (angle < 0.0) {
    angle += kTwoPi;
  }
  while (angle >= kTwoPi) {
    angle -= kTwoPi;
  }
  return angle;
}

double PathPlanner::wrap_to_pi(double angle)
{
  while (angle <= -kPi) {
    angle += kTwoPi;
  }
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  return angle;
}

double PathPlanner::angle_diff_signed(double a, double b)
{
  return wrap_to_pi(a - b);
}

double PathPlanner::approx_goal_bearing_rad(
  const GeoPoint & from,
  const GeoPoint & to)
{
  // Simple local tangent-plane approximation; good enough for nearby waypoints.
  const double lat1 = from.latitude_deg * kPi / 180.0;
  const double lat2 = to.latitude_deg * kPi / 180.0;
  const double dlat = lat2 - lat1;
  const double dlon = (to.longitude_deg - from.longitude_deg) * kPi / 180.0;

  const double x = dlon * std::cos(0.5 * (lat1 + lat2));
  const double y = dlat;

  return wrap_to_2pi(std::atan2(y, x));
}

double PathPlanner::approx_distance_m(
  const GeoPoint & a,
  const GeoPoint & b)
{
  const double lat1 = a.latitude_deg * kPi / 180.0;
  const double lat2 = b.latitude_deg * kPi / 180.0;
  const double dlat = lat2 - lat1;
  const double dlon = (b.longitude_deg - a.longitude_deg) * kPi / 180.0;

  const double x = dlon * std::cos(0.5 * (lat1 + lat2));
  const double y = dlat;
  return kEarthRadiusM * std::sqrt(x * x + y * y);
}

double PathPlanner::goal_potential(
  const PlannerInput & input,
  double candidate_heading_rad) const
{
  // Approximate Pw on the local ring and encourage headings aligned with goal bearing.
  const double goal_bearing = approx_goal_bearing_rad(input.current_position, input.goal_position);
  const double heading_error = std::abs(angle_diff_signed(candidate_heading_rad, goal_bearing));

  // Simple surrogate for UG: lower when aligned toward goal.
  return params_.g_goal * params_.window_size_m * heading_error;
}

double PathPlanner::upwind_potential(
  double candidate_heading_rad,
  double twa_rad) const
{
  const double rel = std::abs(angle_diff_signed(candidate_heading_rad, twa_rad));
  if (rel < params_.phi_up_rad) {
    return params_.g_up * params_.window_size_m;
  }
  return 0.0;
}

double PathPlanner::downwind_potential(
  double candidate_heading_rad,
  double twa_rad) const
{
  const double downwind_dir = wrap_to_2pi(twa_rad + kPi);
  const double rel = std::abs(angle_diff_signed(candidate_heading_rad, downwind_dir));
  if (rel < params_.phi_down_rad) {
    return params_.g_down * params_.window_size_m;
  }
  return 0.0;
}

double PathPlanner::hysteresis_potential(
  double candidate_heading_rad,
  double heading_rad,
  double twa_rad) const
{
  // Basic tack-hysteresis penalty:
  // penalize immediate switching across the wind relative to current heading.
  const double cand_rel = angle_diff_signed(candidate_heading_rad, twa_rad);
  const double curr_rel = angle_diff_signed(heading_rad, twa_rad);

  const bool candidate_on_port = cand_rel > 0.0;
  const bool current_on_port = curr_rel > 0.0;

  const bool candidate_upwind_band =
    std::abs(cand_rel) > params_.phi_up_rad &&
    std::abs(cand_rel) < (kPi - params_.phi_down_rad);

  const bool current_upwind_band =
    std::abs(curr_rel) > params_.phi_up_rad &&
    std::abs(curr_rel) < (kPi - params_.phi_down_rad);

  if (candidate_upwind_band && current_upwind_band && (candidate_on_port != current_on_port)) {
    return params_.g_hyst * params_.window_size_m;
  }

  return 0.0;
}

double PathPlanner::total_potential(
  const PlannerInput & input,
  double candidate_heading_rad) const
{
  return goal_potential(input, candidate_heading_rad) +
         upwind_potential(candidate_heading_rad, input.twa_rad) +
         downwind_potential(candidate_heading_rad, input.twa_rad) +
         hysteresis_potential(candidate_heading_rad, input.heading_rad, input.twa_rad);
}

double PathPlanner::compute_target_heading(const PlannerInput & input) const
{
  const double step_rad = params_.sample_step_deg * kPi / 180.0;

  double best_heading = 0.0;
  double best_potential = std::numeric_limits<double>::infinity();

  for (double h = 0.0; h < kTwoPi; h += step_rad) {
    const double pot = total_potential(input, h);
    if (pot < best_potential) {
      best_potential = pot;
      best_heading = h;
    }
  }

  return wrap_to_2pi(best_heading);
}

}  // namespace mrg_modimoop_planner