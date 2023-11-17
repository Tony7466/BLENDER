// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2016 Michael Rabinovich
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "flip_avoiding_line_search.h"
#include "line_search.h"

#include <Eigen/Dense>
#include <vector>

namespace igl {
namespace flip_avoiding {

inline double get_smallest_pos_quad_zero(double a, double b, double c)
{
  using namespace std;
  double t1, t2;
  if (a != 0) {
    double delta_in = pow(b, 2) - 4 * a * c;
    if (delta_in < 0) {
      return INFINITY;
    }
    double delta = sqrt(delta_in);
    t1 = (-b + delta) / (2 * a);
    t2 = (-b - delta) / (2 * a);
  }
  else {
    t1 = t2 = -b / c;
  }
  assert(std::isfinite(t1));
  assert(std::isfinite(t2));

  double tmp_n = min(t1, t2);
  t1 = max(t1, t2);
  t2 = tmp_n;
  if (t1 == t2) {
    return INFINITY;  // means the orientation flips twice = doesn't flip
  }
  // return the smallest negative root if it exists, otherwise return infinity
  if (t1 > 0) {
    if (t2 > 0) {
      return t2;
    }
    else {
      return t1;
    }
  }
  else {
    return INFINITY;
  }
}

inline double get_min_pos_root_2D(const Eigen::MatrixXd &uv,
                                  const Eigen::MatrixXi &F,
                                  Eigen::MatrixXd &d,
                                  int f)
{
  using namespace std;
  /*
        Finding the smallest timestep t s.t a triangle get degenerated (<=> det = 0)
        The following code can be derived by a symbolic expression in matlab:

        Symbolic matlab:
        U11 = sym('U11');
        U12 = sym('U12');
        U21 = sym('U21');
        U22 = sym('U22');
        U31 = sym('U31');
        U32 = sym('U32');

        V11 = sym('V11');
        V12 = sym('V12');
        V21 = sym('V21');
        V22 = sym('V22');
        V31 = sym('V31');
        V32 = sym('V32');

        t = sym('t');

        U1 = [U11,U12];
        U2 = [U21,U22];
        U3 = [U31,U32];

        V1 = [V11,V12];
        V2 = [V21,V22];
        V3 = [V31,V32];

        A = [(U2+V2*t) - (U1+ V1*t)];
        B = [(U3+V3*t) - (U1+ V1*t)];
        C = [A;B];

        solve(det(C), t);
        cf = coeffs(det(C),t); % Now cf(1),cf(2),cf(3) holds the coefficients for the polynom. at
     order c,b,a
      */

  int v1 = F(f, 0);
  int v2 = F(f, 1);
  int v3 = F(f, 2);
  // get quadratic coefficients (ax^2 + b^x + c)
  const double &U11 = uv(v1, 0);
  const double &U12 = uv(v1, 1);
  const double &U21 = uv(v2, 0);
  const double &U22 = uv(v2, 1);
  const double &U31 = uv(v3, 0);
  const double &U32 = uv(v3, 1);

  const double &V11 = d(v1, 0);
  const double &V12 = d(v1, 1);
  const double &V21 = d(v2, 0);
  const double &V22 = d(v2, 1);
  const double &V31 = d(v3, 0);
  const double &V32 = d(v3, 1);

  double a = V11 * V22 - V12 * V21 - V11 * V32 + V12 * V31 + V21 * V32 - V22 * V31;
  double b = U11 * V22 - U12 * V21 - U21 * V12 + U22 * V11 - U11 * V32 + U12 * V31 + U31 * V12 -
             U32 * V11 + U21 * V32 - U22 * V31 - U31 * V22 + U32 * V21;
  double c = U11 * U22 - U12 * U21 - U11 * U32 + U12 * U31 + U21 * U32 - U22 * U31;

  return get_smallest_pos_quad_zero(a, b, c);
}

inline double compute_max_step_from_singularities(const Eigen::MatrixXd &uv,
                                                  const Eigen::MatrixXi &F,
                                                  Eigen::MatrixXd &d)
{
  using namespace std;
  double max_step = INFINITY;

  // The if statement is outside the for loops to avoid branching/ease parallelizing
  for (int f = 0; f < F.rows(); f++) {
    double min_positive_root = get_min_pos_root_2D(uv, F, d, f);
    max_step = min(max_step, min_positive_root);
  }
  return max_step;
}
}  // namespace flip_avoiding
}  // namespace igl

inline double igl::flip_avoiding_line_search(const Eigen::MatrixXi F,
                                             Eigen::MatrixXd &cur_v,
                                             Eigen::MatrixXd &dst_v,
                                             std::function<double(Eigen::MatrixXd &)> energy,
                                             double cur_energy)
{
  using namespace std;

  Eigen::MatrixXd d = dst_v - cur_v;
  double min_step_to_singularity = igl::flip_avoiding::compute_max_step_from_singularities(
      cur_v, F, d);
  double max_step_size = min(1., min_step_to_singularity * 0.8);
  return igl::line_search(cur_v, d, max_step_size, energy, cur_energy);
}
