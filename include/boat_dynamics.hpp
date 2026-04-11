#pragma once

#include <array>

#include "sail_model.hpp"

namespace sailbot_control
{

constexpr double kBackToMainAlphaRelation = 1.09;

struct BoatParams
{
  double m{0.0};
  double Iz{0.0};
  double rho_w{0.0};
  double rho_a{0.0};
  double S_r{0.0};
  double l_r{0.0};
  double Sf{0.0};
  double CDx{0.0};
  double Yv_coeff{0.0};
  double C_b{0.0};
  double Nv_coeff{0.0};
  double Va{0.0};
  double betaA{0.0};
  double x_ce_m{0.0};
  double y_ce_m{0.0};
  double x_ce_t{0.0};
  double y_ce_t{0.0};
  double CLalpha_r{0.0};
};

struct BodyState
{
  double u{0.0};
  double v{0.0};
  double r{0.0};
};

struct ControlState
{
  double delta_r{0.0};
  double delta_w{0.0};
};

struct ForceMoment
{
  double X{0.0};
  double Y{0.0};
  double N{0.0};
};

struct ForceBreakdown
{
  double alpha_m_deg{0.0};
  double alpha_t_deg{0.0};
  double gamma{0.0};

  SailForces main_sail;
  SailForces back_sail;

  double Xs_m{0.0};
  double Ys_m{0.0};
  double Xs_t{0.0};
  double Ys_t{0.0};

  double X_hull{0.0};
  double Y_hull{0.0};

  double Lr{0.0};
  double Xr_r{0.0};
  double Yr_r{0.0};

  double Ns_m{0.0};
  double Ns_t{0.0};
  double N_r{0.0};
  double N_damp{0.0};
  double N_stiff{0.0};

  ForceMoment total;
};

BoatParams makeDefaultBoatParams();

ForceBreakdown computeForceBreakdown(
  const BodyState & x,
  const ControlState & u,
  const BoatParams & params,
  const SailModel & sail_model);

ForceMoment totalBoatForces(
  const BodyState & x,
  const ControlState & u,
  const BoatParams & params,
  const SailModel & sail_model);

std::array<double, 3> boat3dofRhs(
  const BodyState & x,
  const ControlState & u,
  const BoatParams & params,
  const SailModel & sail_model);

}  // namespace sailbot_control