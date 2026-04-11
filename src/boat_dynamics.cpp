#include "boat_dynamics.hpp"

#include <algorithm>
#include <cmath>

#include "control_utils.hpp"

namespace sailbot_control
{

BoatParams makeDefaultBoatParams()
{
  BoatParams p;
  p.m = 10.0;
  p.Iz = 25.0;
  p.rho_w = 1025.0;
  p.rho_a = 1.225;
  p.S_r = 0.1524 * 0.1524;
  p.l_r = 0.5;
  p.Sf = 0.9 * 1.0 * (0.2 + 2.0 * 0.25);
  p.CDx = 0.6 / 1025.0;
  p.Yv_coeff = 40.0;
  p.C_b = 0.5 * 1025.0 * std::pow(1.143, 5) * 0.01;
  p.Nv_coeff = 0.0;
  p.Va = 4.0;
  p.betaA = M_PI / 2.0;
  p.x_ce_m = 0.26 / 2.0;
  p.y_ce_m = 0.0;
  p.x_ce_t = (1.3 / 2.0) + (0.26 / 2.0);
  p.y_ce_t = 0.0;
  p.CLalpha_r = 5.0;
  return p;
}

ForceBreakdown computeForceBreakdown(
  const BodyState & x,
  const ControlState & u,
  const BoatParams & params,
  const SailModel & sail_model)
{
  ForceBreakdown out;

  out.alpha_m_deg = rad2deg(u.delta_w);
  out.alpha_t_deg = kBackToMainAlphaRelation * rad2deg(-u.delta_w);

  out.main_sail = sail_model.main(out.alpha_m_deg);
  out.back_sail = sail_model.back(out.alpha_t_deg);

  const double Lm = out.main_sail.L;
  const double Dm = out.main_sail.D;
  const double Lt = out.back_sail.L;
  const double Dt = out.back_sail.D;

  const double Va_x = params.Va * std::cos(params.betaA);
  const double Va_y = params.Va * std::sin(params.betaA);

  out.gamma = std::atan2(x.v - Va_y, x.u - Va_x);

  const double Dx_m = -Dm * std::cos(out.gamma);
  const double Dy_m = -Dm * std::sin(out.gamma);
  const double Lx_m = -Lm * std::sin(out.gamma);
  const double Ly_m =  Lm * std::cos(out.gamma);
  out.Xs_m = Lx_m + Dx_m;
  out.Ys_m = Ly_m + Dy_m;

  const double Dx_t = -Dt * std::cos(out.gamma);
  const double Dy_t = -Dt * std::sin(out.gamma);
  const double Lx_t = -Lt * std::sin(out.gamma);
  const double Ly_t =  Lt * std::cos(out.gamma);
  out.Xs_t = Lx_t + Dx_t;
  out.Ys_t = Ly_t + Dy_t;

  out.Ns_m = out.Ys_m * params.x_ce_m + out.Xs_m * params.y_ce_m;
  out.Ns_t = out.Ys_t * params.x_ce_t + out.Xs_t * params.y_ce_t;

  const double D_x = 0.5 * params.rho_w * params.Sf * params.CDx * x.u * std::abs(x.u);
  out.X_hull = -D_x;
  out.Y_hull = -params.Yv_coeff * x.v;

  const double alpha_r = u.delta_r;
  const double q_w = 0.5 * params.rho_w * std::max(x.u * x.u, 0.01);
  out.Lr = q_w * params.S_r * params.CLalpha_r * alpha_r;

  out.Xr_r = 0.0;
  out.Yr_r = out.Lr;

  out.N_damp = -params.C_b * x.r;
  out.N_r = params.l_r * out.Lr;
  out.N_stiff = -params.Nv_coeff * x.v;

  out.total.X = out.Xs_m + out.Xs_t + out.Xr_r + out.X_hull;
  out.total.Y = out.Ys_m + out.Ys_t + out.Yr_r + out.Y_hull;
  out.total.N = out.Ns_m + out.Ns_t + out.N_r + out.N_damp + out.N_stiff;

  return out;
}

ForceMoment totalBoatForces(
  const BodyState & x,
  const ControlState & u,
  const BoatParams & params,
  const SailModel & sail_model)
{
  return computeForceBreakdown(x, u, params, sail_model).total;
}

std::array<double, 3> boat3dofRhs(
  const BodyState & x,
  const ControlState & u,
  const BoatParams & params,
  const SailModel & sail_model)
{
  const auto forces = totalBoatForces(x, u, params, sail_model);

  std::array<double, 3> xdot{};
  xdot[0] = (forces.X / params.m) + x.v * x.r;
  xdot[1] = (forces.Y / params.m) - x.u * x.r;
  xdot[2] = (forces.N / params.Iz);
  return xdot;
}

}  // namespace sailbot_control