// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//      of its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "minimum_snap.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

MinimumSnap::MinimumSnap(const Params & params)
: params_(params) {}

// ---------------------------------------------------------------------------
// Public
// ---------------------------------------------------------------------------

double MinimumSnap::totalTime() const {return total_time_;}

std::vector<Eigen::Vector3d> MinimumSnap::generate(
  const std::vector<Eigen::Vector3d> & waypoints)
{
  // Remove consecutive duplicates to avoid zero-length segments.
  std::vector<Eigen::Vector3d> wps;
  wps.push_back(waypoints.front());
  for (size_t i = 1; i < waypoints.size(); ++i) {
    if ((waypoints[i] - wps.back()).norm() > 1e-6) {
      wps.push_back(waypoints[i]);
    }
  }

  if (wps.size() < 2) {
    // Trivial: start == goal — return a single point
    return {wps.front()};
  }

  n_segs_ = static_cast<int>(wps.size()) - 1;

  // --- Time allocation ---
  // Minimum time per segment from velocity and acceleration constraints.
  // Triangular velocity profile: T = 2*sqrt(d/a_max) and T = d/v_max.
  std::vector<double> dists(n_segs_);
  for (int i = 0; i < n_segs_; ++i) {
    dists[i] = (wps[i + 1] - wps[i]).norm();
  }

  seg_times_.resize(n_segs_);
  for (int i = 0; i < n_segs_; ++i) {
    const double d = dists[i];
    const double t_vel = d / params_.max_vel;
    const double t_acc = 2.0 * std::sqrt(d / params_.max_acc);
    seg_times_[i] = std::max({t_vel, t_acc, 0.5});  // floor at 0.5s
  }

  // Scale to achieve the requested total_time (if the kinematic minimum is less).
  const double t_kin = std::accumulate(seg_times_.begin(), seg_times_.end(), 0.0);
  const double scale = params_.total_time / t_kin;
  if (scale > 1.0) {
    for (auto & t : seg_times_) {t *= scale;}
  }

  total_time_ = std::accumulate(seg_times_.begin(), seg_times_.end(), 0.0);

  // Cumulative start times
  seg_t0_.resize(n_segs_ + 1);
  seg_t0_[0] = 0.0;
  for (int i = 0; i < n_segs_; ++i) {
    seg_t0_[i + 1] = seg_t0_[i] + seg_times_[i];
  }

  // --- Build constraint matrix A (same for all axes) ---
  // 8 coefficients per segment → 8*n_segs_ unknowns per axis.
  // Rows:
  //   [0..3]                  : start boundary (4)
  //   [4 + 8*(j-1) .. 4+8*j-1]: junction j (j=1..N-1), 8 rows each
  //   [4 + 8*(N-1) .. +3]     : end boundary (4)
  const int n = 8 * n_segs_;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);

  int row = 0;

  // --- Start boundary: seg 0, t=0 ---
  // p(0)=w0, p'(0)=0, p''(0)=0, p'''(0)=0
  for (int d = 0; d < 4; ++d) {
    A.row(row).segment<8>(0) = evalRow(d, 0.0);
    ++row;
  }

  // --- Interior junctions ---
  for (int j = 1; j < n_segs_; ++j) {
    const double Ti = seg_times_[j - 1];

    // Right end of segment j-1 = w_j
    A.row(row).segment<8>(8 * (j - 1)) = evalRow(0, Ti);
    ++row;

    // Left end of segment j = w_j
    A.row(row).segment<8>(8 * j) = evalRow(0, 0.0);
    ++row;

    // C6 continuity: derivatives 1..6
    for (int d = 1; d <= 6; ++d) {
      A.row(row).segment<8>(8 * (j - 1)) =  evalRow(d, Ti);
      A.row(row).segment<8>(8 * j)       = -evalRow(d, 0.0);
      ++row;
    }
  }

  // --- End boundary: last seg, t=T_{N-1} ---
  // p(T)=wN, p'(T)=0, p''(T)=0, p'''(T)=0
  {
    const double T_last = seg_times_[n_segs_ - 1];
    for (int d = 0; d < 4; ++d) {
      A.row(row).segment<8>(8 * (n_segs_ - 1)) = evalRow(d, T_last);
      ++row;
    }
  }

  assert(row == n);

  // --- Solve for each axis ---
  const auto decomp = A.colPivHouseholderQr();

  for (int axis = 0; axis < 3; ++axis) {
    Eigen::VectorXd b = Eigen::VectorXd::Zero(n);

    int r = 0;

    // Start
    b(r++) = wps[0](axis);  // position = w0
    // vel=acc=jerk=0 → b stays 0 for rows 1..3

    r += 3;  // skip the 3 zero constraints

    // Junctions
    for (int j = 1; j < n_segs_; ++j) {
      b(r++) = wps[j](axis);  // right end = w_j
      b(r++) = wps[j](axis);  // left end  = w_j
      r += 6;                 // continuity rows → 0 (already zero)
    }

    // End
    b(r++) = wps[n_segs_](axis);  // position = wN
    // vel=acc=jerk=0 → b stays 0 for remaining rows

    coefs_[axis] = decomp.solve(b);
  }

  // --- Sample ---
  std::vector<Eigen::Vector3d> out;
  out.reserve(static_cast<int>(total_time_ / params_.sample_dt) + 2);

  double t = 0.0;
  while (t <= total_time_ + 1e-9) {
    out.push_back(eval(t));
    t += params_.sample_dt;
  }

  return out;
}

Eigen::Vector3d MinimumSnap::eval(double t) const
{
  // Clamp to valid range
  t = std::min(std::max(t, 0.0), total_time_);

  // Find the segment containing t
  int seg = n_segs_ - 1;
  for (int i = 0; i < n_segs_ - 1; ++i) {
    if (t < seg_t0_[i + 1]) {
      seg = i;
      break;
    }
  }

  const double t_local = t - seg_t0_[seg];
  const auto row = evalRow(0, t_local);

  Eigen::Vector3d pt;
  for (int axis = 0; axis < 3; ++axis) {
    pt(axis) = row * coefs_[axis].segment<8>(8 * seg);
  }
  return pt;
}

// ---------------------------------------------------------------------------
// Private
// ---------------------------------------------------------------------------

Eigen::Matrix<double, 1, 8> MinimumSnap::evalRow(int d, double t)
{
  // The d-th derivative of p(t) = c_0 + c_1*t + ... + c_7*t^7 is:
  //   p^(d)(t) = sum_{j=d}^{7} [j!/(j-d)!] * t^(j-d) * c_j
  Eigen::Matrix<double, 1, 8> v = Eigen::Matrix<double, 1, 8>::Zero();
  for (int j = d; j < 8; ++j) {
    double coeff = 1.0;
    for (int k = 0; k < d; ++k) {
      coeff *= static_cast<double>(j - k);
    }
    v(j) = coeff * std::pow(t, j - d);
  }
  return v;
}
