#pragma once

#include <optional>
#include <vector>

namespace mrg_modimoop_planner
{

struct GeoPoint
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
};

struct Point2D
{
  double x_east_m{0.0};
  double y_north_m{0.0};
};

struct PlannerParams
{
  double g_goal{1.0};
  double g_up{5.0};
  double g_down{5.0};
  double g_hyst{2.0};

  double phi_up_rad{0.70};      // upwind exclusion half-angle
  double phi_down_rad{0.52};    // downwind exclusion half-angle

  double window_size_m{5.0};    // Wsize
  double sample_step_deg{5.0};  // 2.5 or 5.0 per design
};

struct PlannerInput
{
  GeoPoint current_position;
  GeoPoint goal_position;

  double heading_rad{0.0};  // boat heading in world frame
  double twa_rad{0.0};      // true wind angle in world frame
};

class PathPlanner
{
public:
  explicit PathPlanner(const PlannerParams & params);

  void set_params(const PlannerParams & params);
  const PlannerParams & params() const;

  // Main API: returns target heading in world frame [0, 2pi)
  double compute_target_heading(const PlannerInput & input) const;

private:
  PlannerParams params_;

  static double wrap_to_2pi(double angle);
  static double wrap_to_pi(double angle);
  static double angle_diff_signed(double a, double b);

  static double approx_goal_bearing_rad(
    const GeoPoint & from,
    const GeoPoint & to);

  static double approx_distance_m(
    const GeoPoint & a,
    const GeoPoint & b);

  double goal_potential(
    const PlannerInput & input,
    double candidate_heading_rad) const;

  double upwind_potential(
    double candidate_heading_rad,
    double twa_rad) const;

  double downwind_potential(
    double candidate_heading_rad,
    double twa_rad) const;

  double hysteresis_potential(
    double candidate_heading_rad,
    double heading_rad,
    double twa_rad) const;

  double total_potential(
    const PlannerInput & input,
    double candidate_heading_rad) const;

  static Point2D geo_to_local_en(
    const GeoPoint & origin,
    const GeoPoint & target);

};

}  // namespace mrg_modimoop_planner