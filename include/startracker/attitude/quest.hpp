#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// quest.hpp — QUEST attitude solver
//
// Solves for spacecraft attitude (ECI → body quaternion) given a set of
// matched star pairs using the QUaternion ESTimator (Shuster & Oh, 1981)
// with iterative outlier rejection for robustness.
//
// ── Pipeline position
// ─────────────────────────────────────────────────────────
//
//   centroids (pixel u,v)
//       │  ST::math::pixel_to_body_ray(u,v, fx,fy, cx,cy)
//       ▼
//   body rays b̂_i   (camera/body frame unit vectors)
//       │  ST::math::AngularSeparationDeg  +  n-star voting
//       ▼
//   confident StarHypothesis  { ray, hip_id }
//       │  [processor.cpp] → MatchedStar list
//       ▼
//   ST::attitude::solve_quest()          ← YOU ARE HERE
//       │
//       ▼
//   AttitudeResult { q, lambda_max, rms_deg, n_stars, valid }
//
// ── Coordinate frames
// ─────────────────────────────────────────────────────────
//
//   Body ray  b̂_i
//     From pixel_to_body_ray() (geometry.cpp, pinhole model):
//         b = normalize( [(u-cx)/fx,  -(v-cy)/fy,  1.0] )
//     X: rightward in image
//     Y: upward  (image row increases downward → flip)
//     Z: forward along optical axis, pointing TOWARD the star
//
//   Reference vector  r̂_i  (ECI / inertial frame)
//     From catalog RA/Dec (both in radians, loaded by catalog::loader):
//         r̂ = [ cos(dec)·cos(ra),  cos(dec)·sin(ra),  sin(dec) ]
//     Points from Earth's centre TOWARD the star.
//
//   Sign convention:
//     Both b̂ and r̂ point TOWARD the star from the observer.
//     For identity attitude (body frame = ECI frame): b̂_i = r̂_i.
//     No negation is applied — they already share the same sense.
//
//   Output quaternion  q
//     Rotates a vector FROM ECI INTO body:
//         b̂_pred = R(q) · r̂_i
//     Eigen storage: Quaterniond coefficients are [x, y, z, w].
//     Constructor signature used: Quaterniond(w, x, y, z).
//
// ── Config values used upstream (not inside the solver) ──────────────────────
//
//   camera_intr_cfg : fx=2522, fy=2522, cx=512, cy=512  (config.json)
//   lookup_cfg      : precision=0.01 deg/bin, tolerance=±5 bins
//
// ─────────────────────────────────────────────────────────────────────────────

#include "startracker/core/types.hpp" // ST::attitude::MatchedStar, AttitudeResult, StarProfileMap
#include <vector>

namespace ST::attitude {

// ─────────────────────────────────────────────────────────────────────────────
// solve_quest
// ─────────────────────────────────────────────────────────────────────────────
//
// Runs QUEST with iterative outlier rejection.
//
// Algorithm overview
// ──────────────────
//   Phase 1 — Initial QUEST solve on all valid (b̂_i, r̂_i) pairs.
//
//   Phase 2 — Outlier rejection loop (max kMaxIter iterations):
//     a. Compute per-star angular residual  θ_i = angle(R·r̂_i,  b̂_i)
//     b. Compute median residual and MAD-based σ̂
//     c. Reject stars with θ_i > median + k·σ̂   (k = 3.0 by default)
//     d. If any stars rejected, re-run QUEST on the surviving inlier set.
//     e. Repeat until no more outliers or fewer than 3 stars remain.
//
//   This handles misidentified stars from the voting step: a wrong HIP ID
//   creates a badly inconsistent (b̂, r̂) pair that drives the residual up.
//   Iterating cleanly removes those pairs before the final solve.
//
// Parameters
// ──────────
//   matched_stars
//     Output of vote_and_hypothesize() filtered to h.confident == true.
//     Each entry:
//       .body_ray — cv::Vec3d normalised body-frame direction
//       (pixel_to_body_ray) .hip_id   — confirmed HIP catalog ID from the
//       voting step
//     Stars with zero body_ray are silently dropped.
//     Stars whose hip_id is absent from catalog are silently dropped.
//     Minimum 3 valid pairs are required; returns invalid otherwise.
//
//   catalog
//     rl_DAT_lookup from main.cpp — maps HIP ID → StarProfile {ra_rad, dec_rad,
//     mag}. RA and Dec must be in radians (catalog::loader handles the
//     conversion).
//
// Returns
// ───────
//   AttitudeResult (startracker/core/types.hpp):
//     .q           — Eigen::Quaterniond, ECI → body  (unit quaternion, qw ≥ 0)
//     .lambda_max  — dominant eigenvalue of K  (→ 1.0 = perfect fit)
//     .rms_deg     — post-fit RMS angular residual in degrees (inlier set only)
//     .n_stars     — number of inlier pairs used in the final solve
//     .valid       — true when lambda_max > 0.5 AND rms_deg < 1.0°
//
AttitudeResult solve_quest(const std::vector<MatchedStar> &matched_stars,
                           const ST::StarProfileMap &catalog);

} // namespace ST::attitude