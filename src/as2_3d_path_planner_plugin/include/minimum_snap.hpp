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

/**
 * @file minimum_snap.hpp
 * @brief Closed-form minimum-snap trajectory generator.
 *
 * Generates a piecewise-polynomial (degree-7) trajectory through a sequence of
 * waypoints. Each axis (X, Y, Z) is solved independently as a linear system
 * A*c = b.  The same constraint matrix A is reused for all three axes.
 *
 * Reference: Mellinger & Kumar, "Minimum snap trajectory generation and
 * control for quadrotors", ICRA 2011 (Ref [32] of the TFM thesis).
 *
 * Constraint structure (8N unknowns for N segments):
 *   Start  : position fixed + vel=acc=jerk=0 (4 constraints)
 *   End    : position fixed + vel=acc=jerk=0 (4 constraints)
 *   Junction i (i=1..N-1) : 2 position + C6 continuity of derivatives 1..6
 *                            = 8 constraints each
 *
 * Time allocation: each segment Ti is proportional to the segment length,
 * clamped to respect max_vel and max_acc (triangular velocity profile).
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef MINIMUM_SNAP_HPP_
#define MINIMUM_SNAP_HPP_

#include <array>
#include <vector>

#include <Eigen/Dense>

/**
 * @brief Tuning parameters for MinimumSnap.
 * Placed at file scope to avoid CWG nested-struct default-argument issue.
 */
struct MinimumSnapParams
{
  double total_time = 10.0;  ///< desired total trajectory duration [s]
  double sample_dt  = 0.05;  ///< sampling step for output and collision check [s]
  double max_vel    = 1.5;   ///< maximum velocity [m/s] for time allocation
  double max_acc    = 2.0;   ///< maximum acceleration [m/s²] for time allocation
};

class MinimumSnap
{
public:
  using Params = MinimumSnapParams;

  explicit MinimumSnap(const Params & params = Params{});

  /**
   * @brief Generate a smooth trajectory through the given waypoints.
   *
   * @param waypoints Ordered list of 3D positions.  Must have ≥ 2 entries.
   * @return Vector of sampled positions at `params.sample_dt` intervals
   *         (from t=0 to t=totalTime(), inclusive).  Empty on failure.
   */
  std::vector<Eigen::Vector3d> generate(
    const std::vector<Eigen::Vector3d> & waypoints);

  /**
   * @brief Evaluate the trajectory at global time t ∈ [0, totalTime()].
   *
   * Must be called after a successful generate().
   */
  Eigen::Vector3d eval(double t) const;

  /// Total trajectory duration in seconds (set after generate()).
  double totalTime() const;

private:
  /// Evaluate the row vector for p^(d)(t) in terms of 8 polynomial coefficients.
  static Eigen::Matrix<double, 1, 8> evalRow(int d, double t);

  Params params_;

  int n_segs_{0};
  double total_time_{0.0};
  std::vector<double> seg_times_;    ///< T_i for each segment
  std::vector<double> seg_t0_;       ///< cumulative start time of each segment

  /// Polynomial coefficients per axis:  coefs_[axis] has length 8*n_segs_
  std::array<Eigen::VectorXd, 3> coefs_;
};

#endif  // MINIMUM_SNAP_HPP_
