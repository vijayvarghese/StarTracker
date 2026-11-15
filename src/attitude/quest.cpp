#include "startracker/attitude/quest.hpp"
#include "startracker/core/logger.hpp"
#include "startracker/core/types.hpp"
#include "startracker/util/dbg/debug_utils.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace ST::attitude {
namespace {

using Pair = std::pair<Eigen::Vector3d, Eigen::Vector3d>; // (body_ray, eci_ray)

/**
 * Converts Right Ascension and Declination into a Right-Handed ECI unit vector.
 * X points to Vernal Equinox (RA=0, Dec=0). Z points to North Pole (Dec=+90).
 */
static Eigen::Vector3d catalog_to_eci(double ra_rad, double dec_rad) {
  return Eigen::Vector3d(std::cos(dec_rad) * std::cos(ra_rad),
                         std::cos(dec_rad) * std::sin(ra_rad),
                         std::sin(dec_rad))
      .normalized();
}

/**
 * Builds the attitude profile matrix B from the body and ECI vector pairs.
 * B = Sum(w_i * b_i * r_i^T)
 */
static Eigen::Matrix3d build_B(const std::vector<Pair> &pairs) {
  Eigen::Matrix3d B = Eigen::Matrix3d::Zero();
  const double w = 1.0 / static_cast<double>(pairs.size());
  for (const auto &[b, r] : pairs) {
    B += w * (b * r.transpose());
  }
  return B;
}

/**
 * Constructs Davenport's K-matrix (4x4) from the B matrix.
 * The K-matrix translates Wahba's problem into an eigenvalue problem.
 */
static Eigen::Matrix4d build_K(const Eigen::Matrix3d &B) {
  const double sigma = B.trace();
  const Eigen::Matrix3d S = B + B.transpose();

  // Z is the anti-symmetric part of B
  const Eigen::Vector3d Z(B(1, 2) - B(2, 1), B(2, 0) - B(0, 2),
                          B(0, 1) - B(1, 0));

  Eigen::Matrix4d K = Eigen::Matrix4d::Zero();
  K.topLeftCorner<3, 3>() = S - sigma * Eigen::Matrix3d::Identity();
  K.topRightCorner<3, 1>() = Z;
  K.bottomLeftCorner<1, 3>() = Z.transpose();
  K(3, 3) = sigma;

  return K;
}

/**
 * Extracts the optimal quaternion from Davenport's K-matrix.
 * The optimal quaternion is the eigenvector corresponding to the largest
 * eigenvalue.
 */
static Eigen::Quaterniond extract_quat(const Eigen::Matrix4d &K,
                                       double &lambda_max_out) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(K);
  lambda_max_out = solver.eigenvalues()(3);

  Eigen::Vector4d ev = solver.eigenvectors().col(3);
  ev.normalize();

  // Enforce positive scalar part (w) for uniqueness
  if (ev(3) < 0.0) {
    ev = -ev;
  }

  // Apply conjugate (-x, -y, -z) to convert Davenport's JPL convention
  // into Eigen's Hamilton convention!
  return Eigen::Quaterniond(ev(3), -ev(0), -ev(1), -ev(2));
}

/**
 * Calculates the angular error (in degrees) between the expected body vector
 * and the actual measured body vector for each star.
 */
static std::vector<double> per_star_residuals(const Eigen::Quaterniond &q,
                                              const std::vector<Pair> &pairs) {
  const Eigen::Matrix3d R = q.toRotationMatrix();
  std::vector<double> res;
  res.reserve(pairs.size());

  for (const auto &[b, r] : pairs) {
    // R * r rotates the ECI vector into the Camera Body frame.
    // We then find the angle between the expected body ray and measured body
    // ray.
    const double dot = std::clamp((R * r).dot(b), -1.0, 1.0);
    res.push_back(std::acos(dot) * (180.0 / std::numbers::pi));
  }
  return res;
}

static double rms_of(const std::vector<double> &v) {
  if (v.empty())
    return 0.0;
  double s = 0.0;
  for (double x : v)
    s += x * x;
  return std::sqrt(s / static_cast<double>(v.size()));
}

/**
 * Calculates a dynamic threshold for outlier rejection using
 * the Median Absolute Deviation (MAD) of the residuals.
 */
static double outlier_threshold_deg(std::vector<double> v) {
  if (v.empty())
    return 0.0;

  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  const double med = (n % 2 == 0) ? 0.5 * (v[n / 2 - 1] + v[n / 2]) : v[n / 2];

  std::vector<double> dev(n);
  for (size_t i = 0; i < n; ++i) {
    dev[i] = std::abs(v[i] - med);
  }

  std::sort(dev.begin(), dev.end());
  const double mad =
      (n % 2 == 0) ? 0.5 * (dev[n / 2 - 1] + dev[n / 2]) : dev[n / 2];

  constexpr double kMinThresh = 0.5; // absolute minimum threshold in degrees
  return std::max(med + 3.0 * 1.4826 * mad, kMinThresh);
}

} // anonymous namespace

AttitudeResult solve_quest(const std::vector<MatchedStar> &matched_stars,
                           const ST::StarProfileMap &catalog) {
  // ── Phase 1: Build valid measurement/reference pairs ──────────────────────
  std::vector<Pair> pairs;
  pairs.reserve(matched_stars.size());

  for (const auto &star : matched_stars) {
    // Skip invalid rays
    if (cv::norm(star.body_ray) < 1e-6)
      continue;

    auto it = catalog.find(star.hip_id);
    if (it == catalog.end())
      continue;

    // b: Body-frame unit vector (Right-Handed: +X Right, +Y Down, +Z Fwd)
    Eigen::Vector3d b(star.body_ray[0], star.body_ray[1], star.body_ray[2]);
    b.normalize();

    // r: ECI unit vector (Right-Handed: +X Vernal Equinox, +Z North Pole)
    const Eigen::Vector3d r =
        catalog_to_eci(it->second.ra_rad, it->second.dec_rad);

    if (ST::dbg::quest) {
      LOG_DEBUG << "[QUEST] HIP=" << star.hip_id << std::fixed
                << std::setprecision(4) << "  ra=" << it->second.ra_rad
                << "  dec=" << it->second.dec_rad << "  b·r=" << b.dot(r)
                << "  b=[" << b.transpose() << "]"
                << "  r=[" << r.transpose() << "]";
    }

    pairs.emplace_back(b, r);
  }

  // ── Minimum pair guard ────────────────────────────────────────────────────
  // A minimum of 2 stars is mathematically required for 3-axis attitude,
  // but 3 is the standard engineering minimum for basic confidence.
  if (pairs.size() < 3) {
    if (ST::dbg::quest)
      LOG_DEBUG << "[QUEST] Too few valid pairs: " << pairs.size()
                << " < 3. Returning invalid.";
    return AttitudeResult{Eigen::Quaterniond::Identity(), 0.0, 0.0,
                          static_cast<int>(pairs.size()), false};
  }

  // ── Phase 2: Iterative q-method + MAD Outlier Rejection ───────────────────
  static constexpr int kMaxIter = 5;

  std::vector<Pair> active = pairs;
  double lambda_max = 0.0;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  double rms_deg = 0.0;

  for (int iter = 0; iter < kMaxIter; ++iter) {
    // Davenport's q-method Core
    const Eigen::Matrix3d B = build_B(active);
    const Eigen::Matrix4d K = build_K(B);
    q = extract_quat(K, lambda_max);

    // Evaluate solution quality
    std::vector<double> residuals = per_star_residuals(q, active);
    rms_deg = rms_of(residuals);

    if (ST::dbg::quest) {
      LOG_DEBUG << "[QUEST] iter=" << iter << "  n=" << active.size()
                << "  sigma=" << B.trace() << "  lambda=" << lambda_max
                << "  rms=" << rms_deg << " deg";
    }

    // Outlier rejection
    const double thresh = outlier_threshold_deg(residuals);
    std::vector<Pair> survivors;
    survivors.reserve(active.size());
    int n_rejected = 0;

    for (size_t i = 0; i < active.size(); ++i) {
      if (residuals[i] <= thresh) {
        survivors.push_back(active[i]);
      } else {
        ++n_rejected;
        if (ST::dbg::quest) {
          LOG_DEBUG << "[QUEST]   REJECT star " << i
                    << " theta=" << residuals[i] << " > thresh=" << thresh;
        }
      }
    }

    if (n_rejected == 0)
      break; // Converged: no more outliers
    if (survivors.size() < 3)
      break; // Too few remaining: keep last valid result

    active = std::move(survivors);
  }

  // ── Phase 3: Final Validation & Return ────────────────────────────────────
  // lambda_max should approach 1.0 for a perfect match (normalized weights).
  // rms_deg threshold ensures we don't return garbage geometry matches.
  const bool valid = (lambda_max > 0.5) && (rms_deg < 1.0);

  if (ST::dbg::quest) {
    LOG_DEBUG << "[QUEST] FINAL"
              << "  n_inliers=" << active.size()
              << "  lambda_max=" << lambda_max << "  rms_deg=" << rms_deg
              << "  valid=" << std::boolalpha << valid << "  q=[w=" << q.w()
              << " x=" << q.x() << " y=" << q.y() << " z=" << q.z() << "]";
  }

  return AttitudeResult{q, lambda_max, rms_deg, static_cast<int>(active.size()),
                        valid};
}

} // namespace ST::attitude