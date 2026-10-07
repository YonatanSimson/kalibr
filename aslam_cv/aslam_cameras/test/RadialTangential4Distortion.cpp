// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras.hpp>
#include <aslam/cameras/FiniteDifferences.hpp>
#include <aslam/cameras/RadialTangential4Distortion.hpp>
#include <aslam/cameras/test/CameraGeometryTestHarness.hpp>

namespace {
// Insta360 X6 factory-like terms (offset_v6 lens 0) plus small tangential ones.
aslam::cameras::RadialTangential4Distortion x6Like() {
  return aslam::cameras::RadialTangential4Distortion(1.302838, -0.993135, -0.0009, 0.0007,
                                                     2.559152, 9.183159);
}
}  // namespace

TEST(AslamCamerasTestSuite, testRT4DistortionParameterJacobian)
{
  using namespace aslam::cameras;
  RadialTangential4Distortion d = x6Like();
  for (double r : {4e-3, 0.1, 0.3, 0.45}) {
    Eigen::Vector2d k(0.6 * r, -0.8 * r);
    Eigen::Vector2d kd = k;
    Eigen::MatrixXd Jd, estJd;
    d.distortParameterJacobian(kd, Jd);
    ASLAM_CAMERAS_ESTIMATE_DISTORTION_JACOBIAN(distort, d, k, 1e-5, estJd);
    ASSERT_EQ(6, Jd.cols());
    // Absolute + relative: near the axis the r^4..r^8 columns (1e-13..1e-22) are below what a
    // finite difference resolves, so a purely relative check fails on rounding noise.
    for (int row = 0; row < 2; ++row) {
      for (int col = 0; col < 6; ++col) {
        EXPECT_NEAR(Jd(row, col), estJd(row, col), 1e-9 + 1e-6 * std::fabs(Jd(row, col)))
            << "radius " << r << ", row " << row << ", col " << col;
      }
    }
  }
}

TEST(AslamCamerasTestSuite, testRT4DistortionPointJacobian)
{
  using namespace aslam::cameras;
  RadialTangential4Distortion d = x6Like();
  const double h = 1e-6;
  for (double r : {4e-3, 0.1, 0.3, 0.45}) {
    Eigen::Vector2d y0(0.6 * r, -0.8 * r);
    Eigen::Vector2d y = y0;
    Eigen::Matrix2d J;
    d.distort(y, J);
    Eigen::Matrix2d Jfd;
    for (int c = 0; c < 2; ++c) {
      Eigen::Vector2d a = y0, b = y0;
      a[c] += h;
      b[c] -= h;
      d.distort(a);
      d.distort(b);
      Jfd.col(c) = (a - b) / (2.0 * h);
    }
    ASSERT_DOUBLE_MX_EQ(J, Jfd, 0.01, "");
  }
}

TEST(AslamCamerasTestSuite, testRT4DistortionRoundTrip)
{
  using namespace aslam::cameras;
  RadialTangential4Distortion d = x6Like();
  // Out to the X6 rim (normalised MEI radius ~0.45 at 96 deg with xi 2.455).
  for (double r : {1e-3, 0.1, 0.2, 0.3, 0.4, 0.45}) {
    for (double a : {0.0, 1.0, 2.5, 4.0}) {
      Eigen::Vector2d y0(r * std::cos(a), r * std::sin(a));
      Eigen::Vector2d y = y0;
      d.distort(y);
      d.undistort(y);
      EXPECT_NEAR(y0[0], y[0], 1e-9);
      EXPECT_NEAR(y0[1], y[1], 1e-9);
    }
  }
}

TEST(AslamCamerasTestSuite, testRadtan4DistortedOmniCameraGeometry)
{
  using namespace aslam::cameras;
  Radtan4DistortedOmniCameraGeometry geometry = Radtan4DistortedOmniCameraGeometry::getTestGeometry();
  geometry.projection().setDistortion(RadialTangential4Distortion::getTestDistortion());
  CameraGeometryTestHarness<Radtan4DistortedOmniCameraGeometry> harness(geometry, 0.5);
  SCOPED_TRACE("radtan4 distorted omni");
  harness.testAll();
}
