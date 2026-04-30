#include "path_planner.hpp"

#include <cmath>
#include <limits>

namespace mrg_modimoop_planner
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
}  // namespace

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
  const Vector3D & from,
  const Vector3D & to)
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;

  return wrap_to_2pi(std::atan2(dy, dx));
}

double PathPlanner::goal_potential(
  const PlannerInput & input,
  double candidate_heading_rad) const
{
  const double goal_bearing = approx_goal_bearing_rad(input.current_position, input.goal_position);
  const double heading_error = std::abs(angle_diff_signed(candidate_heading_rad, goal_bearing));

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
  // twa is relative to the robot's frame, not world frame
  const double wind_world_rad = wrap_to_2pi(input.heading_rad + input.twa_rad);

  return goal_potential(input, candidate_heading_rad) +
         upwind_potential(candidate_heading_rad, wind_world_rad) +
         downwind_potential(candidate_heading_rad, wind_world_rad) +
         hysteresis_potential(candidate_heading_rad, input.heading_rad, wind_world_rad);
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

PlannerDebugInfo PathPlanner::compute_debug_info(const PlannerInput & input) const
{
  PlannerDebugInfo info;

  info.target_heading_rad = compute_target_heading(input);
  info.heading_rad = input.heading_rad;
  info.twa_relative_rad = input.twa_rad;
  info.wind_world_rad = wrap_to_2pi(input.heading_rad + input.twa_rad);
  info.goal_potential = goal_potential(input, info.target_heading_rad);
  info.upwind_potential = goal_potential(input, info.target_heading_rad);
  info.downwind_potential = downwind_potential(info.target_heading_rad, info.wind_world_rad);
  info.hysteresis_potential = hysteresis_potential(
    info.target_heading_rad,
    input.heading_rad,
    info.wind_world_rad);
  info.total_potential = info.goal_potential + info.upwind_potential + info.downwind_potential + info.hysteresis_potential;

  return info;
}

}  // namespace mrg_modimoop_planner