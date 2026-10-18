// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras.hpp>
#include <aslam/cameras/FiniteDifferences.hpp>
#include <aslam/cameras/RadialTangentialProDistortion.hpp>
#include <aslam/cameras/test/CameraGeometryTestHarness.hpp>

namespace {
using aslam::cameras::RadialTangentialProDistortion;

// Insta360 X6 BXEA3ABHFQ7YSX offset_v6 lens 0, d0..d12, with d4 (0 on this unit) set non-zero
// so the r^10 column is exercised.
std::vector<double> x6Coefficients() {
  return {1.30283809, -0.99313498, 2.55915213, 9.18315887, 0.5, -0.00097495, 0.000738,
          -0.00091928, 0.01121711, 0.00068693, -0.00096601, -0.00637395, -0.02075081};
}

// telemetry-parser `MeiDistortion::RadtanPro` (src/insta360/rig_body_masks.rs), verbatim.
Eigen::Vector2d vendorForm(const std::vector<double> & d, double x, double y) {
  const double r2 = x * x + y * y;
  const double radial = 1. + r2 * (d[0] + r2 * (d[1] + r2 * (d[2] + r2 * (d[3] + r2 * d[4]))));
  const double a = d[5] + d[7] * r2;
  const double b = d[6] + d[8] * r2;
  return Eigen::Vector2d(
      x * radial + (r2 + 2. * x * x) * a + 2. * x * y * b + d[9] * r2 + d[11] * r2 * r2,
      y * radial + (r2 + 2. * y * y) * b + 2. * x * y * a + d[10] * r2 + d[12] * r2 * r2);
}

// Restores "nothing fixed" however a test exits.
struct ScopedFixedTerms {
  explicit ScopedFixedTerms(const std::vector<int> & indices) {
    aslam::cameras::radtanpro::setFixed(indices);
  }
  ~ScopedFixedTerms() { aslam::cameras::radtanpro::setFixed({}); }
};

const double kRadii[] = {4e-3, 0.1, 0.3, 0.45};  // 0.45 ~ the X6 rim in MEI-normalised units
}  // namespace

TEST(AslamCamerasTestSuite, testRTProMatchesVendorForm)
{
  RadialTangentialProDistortion dist(x6Coefficients());
  for (double r : kRadii) {
    for (double angle : {0.0, 0.7, 2.0, 3.9, 5.5}) {
      Eigen::Vector2d y(r * std::cos(angle), r * std::sin(angle));
      const Eigen::Vector2d expected = vendorForm(x6Coefficients(), y[0], y[1]);
      dist.distort(y);
      EXPECT_NEAR(expected[0], y[0], 1e-14);
      EXPECT_NEAR(expected[1], y[1], 1e-14);
    }
  }
}

TEST(AslamCamerasTestSuite, testRTProParameterJacobian)
{
  ScopedFixedTerms none({});
  RadialTangentialProDistortion dist(x6Coefficients());
  for (double r : kRadii) {
    Eigen::Vector2d k(0.6 * r, -0.8 * r);
    Eigen::Vector2d kd = k;
    Eigen::MatrixXd Jd, estJd;
    dist.distortParameterJacobian(kd, Jd);
    ASLAM_CAMERAS_ESTIMATE_DISTORTION_JACOBIAN(distort, dist, k, 1e-5, estJd);
    ASSERT_EQ(13, Jd.cols());
    // Absolute + relative, as radtan4: near the axis the high-order columns are below what a
    // finite difference resolves.
    for (int row = 0; row < 2; ++row) {
      for (int col = 0; col < 13; ++col) {
        EXPECT_NEAR(Jd(row, col), estJd(row, col), 1e-9 + 1e-6 * std::fabs(Jd(row, col)))
            << "radius " << r << ", row " << row << ", col " << col;
      }
    }
  }
}

TEST(AslamCamerasTestSuite, testRTProPointJacobian)
{
  RadialTangentialProDistortion dist(x6Coefficients());
  const double h = 1e-6;
  for (double r : kRadii) {
    for (double angle : {0.3, 2.2, 4.4}) {
      Eigen::Vector2d y0(r * std::cos(angle), r * std::sin(angle));
      Eigen::Vector2d y = y0;
      Eigen::Matrix2d J;
      dist.distort(y, J);
      Eigen::Matrix2d Jfd;
      for (int c = 0; c < 2; ++c) {
        Eigen::Vector2d a = y0, b = y0;
        a[c] += h;
        b[c] -= h;
        dist.distort(a);
        dist.distort(b);
        Jfd.col(c) = (a - b) / (2.0 * h);
      }
      for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
          EXPECT_NEAR(J(i, j), Jfd(i, j), 1e-7) << "radius " << r << ", angle " << angle;
        }
      }
    }
  }
}

TEST(AslamCamerasTestSuite, testRTProRoundTrip)
{
  RadialTangentialProDistortion dist(x6Coefficients());
  for (double r : {1e-3, 0.1, 0.2, 0.3, 0.4, 0.45}) {
    for (double angle : {0.0, 1.0, 2.5, 4.0}) {
      Eigen::Vector2d y0(r * std::cos(angle), r * std::sin(angle));
      Eigen::Vector2d y = y0;
      dist.distort(y);
      dist.undistort(y);
      EXPECT_NEAR(y0[0], y[0], 1e-9);
      EXPECT_NEAR(y0[1], y[1], 1e-9);
    }
  }
}

TEST(AslamCamerasTestSuite, testRTProFixedTerms)
{
  const std::vector<int> fixedTerms = {4, 7, 8};
  RadialTangentialProDistortion dist(x6Coefficients());

  Eigen::Vector2d k(0.18, -0.24);
  Eigen::MatrixXd Jfull;
  {
    ScopedFixedTerms none({});
    dist.distortParameterJacobian(k, Jfull);
  }
  ScopedFixedTerms scoped(fixedTerms);
  ASSERT_EQ(10, dist.minimalDimensions());

  Eigen::MatrixXd J;
  dist.distortParameterJacobian(k, J);
  ASSERT_EQ(2, J.rows());
  ASSERT_EQ(10, J.cols());
  int col = 0;
  for (int i = 0; i < 13; ++i) {
    if (i == 4 || i == 7 || i == 8) {
      continue;
    }
    EXPECT_EQ(Jfull(0, i), J(0, col));
    EXPECT_EQ(Jfull(1, i), J(1, col));
    ++col;
  }

  // update() moves the free terms in order and leaves the fixed ones alone.
  Eigen::MatrixXd P0, P1;
  dist.getParameters(P0);
  ASSERT_EQ(13, P0.rows());
  double v[10];
  for (int i = 0; i < 10; ++i) {
    v[i] = 0.1 * (i + 1);
  }
  dist.update(v);
  dist.getParameters(P1);
  int free = 0;
  for (int i = 0; i < 13; ++i) {
    if (i == 4 || i == 7 || i == 8) {
      EXPECT_EQ(P0(i), P1(i)) << "coefficient " << i;
    } else {
      EXPECT_NEAR(P0(i) + v[free], P1(i), 1e-12) << "coefficient " << i;
      ++free;
    }
  }
}

TEST(AslamCamerasTestSuite, testRadtanProDistortedOmniCameraGeometry)
{
  using namespace aslam::cameras;
  ScopedFixedTerms none({});
  RadtanProDistortedOmniCameraGeometry geometry = RadtanProDistortedOmniCameraGeometry::getTestGeometry();
  geometry.projection().setDistortion(RadialTangentialProDistortion::getTestDistortion());
  CameraGeometryTestHarness<RadtanProDistortedOmniCameraGeometry> harness(geometry, 0.5);
  SCOPED_TRACE("radtanpro distorted omni");
  harness.testAll();
}
