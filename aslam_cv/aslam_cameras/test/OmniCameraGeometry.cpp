// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras.hpp>
#include <aslam/cameras/test/CameraGeometryTestHarness.hpp>

TEST(AslamCamerasTestSuite, testOmniCameraGeometry)
{
  using namespace aslam::cameras;
  // Use a forgiving tolerance because of the approximate
  // undistortion used in this model.
  CameraGeometryTestHarness<OmniCameraGeometry> harness(0.1);

  SCOPED_TRACE("omni camera");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDistortedOmniCameraGeometryDefault)
{
  using namespace aslam::cameras;
  // Use a forgiving tolerance because of the approximate
  // undistortion used in this model.
  CameraGeometryTestHarness<DistortedOmniCameraGeometry> harness(0.5);

  SCOPED_TRACE("distorted omni default");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDistortedOmniCameraGeometry)
{
  using namespace aslam::cameras;
  // Use a forgiving tolerance because of the approximate
  // undistortion used in this model.
  RadialTangentialDistortion d(-0.2, 0.13, 0.0005, 0.0005);
  DistortedOmniCameraGeometry geometry = DistortedOmniCameraGeometry::getTestGeometry();
  geometry.projection().setDistortion(d);
  CameraGeometryTestHarness<DistortedOmniCameraGeometry> harness(geometry,0.5);

  SCOPED_TRACE("distorted omni");
  harness.testAll();

}

// KALIBR_OMNI_XI_FIXED (OMNI_FIXED_XI.md): xi leaves the parameter block, which becomes
// [fu fv cu cv]. Checked at several xi on both sides of 1 (the MEI liftable region changes there).
namespace {

// Restores the process-wide fixed-xi flag when the test ends.
struct ScopedFixedXi {
  bool previous;
  explicit ScopedFixedXi(bool f) : previous(aslam::cameras::omni_xi::fixed()) {
    aslam::cameras::omni_xi::setFixed(f);
  }
  ~ScopedFixedXi() { aslam::cameras::omni_xi::setFixed(previous); }
};

template<typename GEOMETRY_T>
void checkFixedXi(GEOMETRY_T geometry) {
  using namespace aslam::cameras;
  ScopedFixedXi scoped(true);
  for (double xi : {0.5, 1.0, 2.0, 2.5, 3.0}) {
    SCOPED_TRACE(::testing::Message() << "xi " << xi);
    Eigen::MatrixXd P;
    geometry.projection().getParameters(P);
    P(0) = xi;
    geometry.projection().setParameters(P);

    ASSERT_EQ(4, geometry.projection().minimalDimensions());
    ASSERT_EQ(4 + geometry.projection().distortion().minimalDimensions(),
              geometry.minimalDimensions(true, true, false));

    // Points from random pixels that lift and re-project (createRandomVisiblePoint divides by
    // xi^2 - 1 and never returns at xi = 1). For xi > 1 this stays inside the liftable region.
    int checked = 0;
    for (int attempt = 0; attempt < 5000 && checked < 20; ++attempt) {
      Eigen::Vector2d kp((double) rand() / RAND_MAX * geometry.projection().ru(),
                         (double) rand() / RAND_MAX * geometry.projection().rv());
      Eigen::Vector3d p;
      Eigen::VectorXd kp2;
      if (!geometry.keypointToEuclidean(kp, p) || !geometry.euclideanToKeypoint(p, kp2)
          || (kp2 - kp).norm() > 1e-6) {
        continue;
      }
      p = 2.0 * p.normalized();
      ++checked;

      Eigen::MatrixXd J, J5, Jfd;
      geometry.euclideanToKeypointIntrinsicsJacobian(p, J, true, false, false);
      ASSERT_EQ(2, J.rows());
      ASSERT_EQ(4, J.cols());
      geometry.euclideanToKeypointIntrinsicsJacobianFiniteDifference(p, Jfd);  // d/d[xi fu fv cu cv]
      Eigen::MatrixXd Jref = Jfd.rightCols(4);
      ASSERT_DOUBLE_MX_EQ(J, Jref, 0.5, "");  // percent, as in the harness

      // The fixed-xi block is the free-xi Jacobian without its xi column.
      omni_xi::setFixed(false);
      geometry.euclideanToKeypointIntrinsicsJacobian(p, J5, true, false, false);
      omni_xi::setFixed(true);
      ASSERT_EQ(5, J5.cols());
      Eigen::MatrixXd J5ref = J5.rightCols(4);
      ASSERT_DOUBLE_MX_EQ(J, J5ref, 1e-10, "");
    }
    ASSERT_EQ(20, checked);

    // update() must leave xi alone and move fu fv cu cv.
    Eigen::MatrixXd P0, P1;
    geometry.projection().getParameters(P0);
    const double v[4] = {1.0, 2.0, 3.0, 4.0};
    geometry.projection().update(v);
    geometry.projection().getParameters(P1);
    EXPECT_EQ(xi, P1(0));
    for (int i = 0; i < 4; ++i) {
      EXPECT_NEAR(P0(i + 1) + v[i], P1(i + 1), 1e-12);
    }
  }
}

}  // namespace

TEST(AslamCamerasTestSuite, testDistortedOmniFixedXi)
{
  using namespace aslam::cameras;
  DistortedOmniCameraGeometry geometry = DistortedOmniCameraGeometry::getTestGeometry();
  geometry.projection().setDistortion(RadialTangentialDistortion(1.17, 0.28, -0.003, 0.003));
  checkFixedXi(geometry);
}

TEST(AslamCamerasTestSuite, testRadtan4DistortedOmniFixedXi)
{
  using namespace aslam::cameras;
  Radtan4DistortedOmniCameraGeometry geometry = Radtan4DistortedOmniCameraGeometry::getTestGeometry();
  geometry.projection().setDistortion(RadialTangential4Distortion::getTestDistortion());
  checkFixedXi(geometry);
}
