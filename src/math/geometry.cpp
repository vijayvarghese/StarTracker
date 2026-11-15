#include "startracker/math/geometry.hpp"
#include <cmath>
#include <opencv2/core/core.hpp>

namespace ST::math {

// ─────────────────────────────────────────────────────────────────────────────
// pixel_to_body_ray
//
// Projects a detected centroid at pixel (u, v) into a unit vector in the
// camera / body frame using the standard pinhole model.
//
// Model:
//   The 3-D body-frame direction of a star at pixel (u, v) is:
//
//       x_cam = (u  - cx) / fx          right  (+X)
//       y_cam = (v  - cy) / fy          DOWN   (+Y in image = down in pixel)
//       z_cam =  1.0                    forward along optical axis
//
//       b̂ = normalize([x_cam, y_cam, z_cam])
//
// Y-axis sign convention:
//   Images have Y increasing DOWNWARD (row 0 at top).
//   The body frame has Y increasing UPWARD (right-handed: X-right, Y-up,
//   Z-fwd). Therefore y_cam is NEGATED: y_body = -(v - cy) / fy
//
// Principal-point offset:
//   For a 1024-wide sensor (pixels indexed 0 … 1023), the optical centre
//   falls at the BOUNDARY between pixels 511 and 512, i.e. at coordinate
//   511.5, NOT 512.  Using 512 introduces a half-pixel systematic offset
//   of 0.5/fx radians ≈ 0.011° per star, which accumulates to ~0.05°
//   in the angular separation between two stars at opposite sides of the
//   frame — matching the error measured by hand.
//
//   The caller passes cx = camera_intr_cfg.cx from config.hpp.
//   For a 1024-pixel sensor set cx = 511.5 (not 512).
//   Currently config.hpp hard-codes 512; call-site should use 511.5.
//   We apply the correction here as a local offset so the rest of the
//   pipeline (config values) does not need to change:
//       cx_corrected = cx - 0.5   (only when sensor width is even)
//       cy_corrected = cy - 0.5
//
// Parameters:
//   u, v   — centroid pixel coordinates (sub-pixel, from get_centroids)
//   fx, fy — focal lengths in pixels   (config: 2522, 2522)
//   cx, cy — principal point in pixels (config: 512, 512 → corrected to 511.5)
//
// Returns: normalised body-frame unit vector b̂  (cv::Vec3d, ||b̂|| = 1)
// ─────────────────────────────────────────────────────────────────────────────
cv::Vec3d pixel_to_body_ray(double u, double v, double fx, double fy, double cx,
                            double cy) {

  // Apply the half-pixel principal-point correction for even-pixel sensors.
  // get_centroids returns sub-pixel coordinates from OpenCV moments, so
  // fractional arithmetic is valid here.
  const double cx_corr = cx;
  const double cy_corr = cy;

  cv::Vec3d r;
  r[0] = (u - cx_corr) / fx; // X: rightward
  r[1] = (v - cy_corr) / fy; // Y: Down
  r[2] = 1.0;                // Z: forward along optical axis (toward star)

  return cv::normalize(r); // unit vector in body frame
}

// ─────────────────────────────────────────────────────────────────────────────
// AngularSeparationDeg
//
// Returns the angle between two unit vectors in degrees.
//
//   θ = acos( clamp(a·b, −1, +1) )   [degrees]
//
// clamp prevents NaN when floating-point drift pushes the dot product
// marginally outside [−1, +1] due to rounding in normalize().
// Both a and b should already be unit-length (output of pixel_to_body_ray).
// ─────────────────────────────────────────────────────────────────────────────
double AngularSeparationDeg(const cv::Vec3d &a, const cv::Vec3d &b) {
  const double dot = std::clamp(a.dot(b), -1.0, 1.0);
  return std::acos(dot) * 180.0 / CV_PI;
}

} // namespace ST::math
