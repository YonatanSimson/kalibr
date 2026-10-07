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

// KALIBR_OMNI_XI_FIXED=1 (OMNI_FIXED_XI.md): xi leaves the parameter block, which becomes
// [fu fv cu cv]. Runs only with the variable set -- the flag is read once per process, so run it
// as its own process:
//   KALIBR_OMNI_XI_FIXED=1 aslam_cameras-test --gtest_filter='*FixedXi*'
// (the generic harness tests compare against a 5-parameter finite difference and do not apply).
TEST(AslamCamerasTestSuite, testDistortedOmniFixedXi)
{
  using namespace aslam::cameras;
  if (!omni_xi::fixed()) {
    GTEST_SKIP() << "needs KALIBR_OMNI_XI_FIXED=1";
  }
  RadialTangentialDistortion d(1.17, 0.28, -0.003, 0.003);
  DistortedOmniCameraGeometry geometry = DistortedOmniCameraGeometry::getTestGeometry();
  geometry.projection().setDistortion(d);
  ASSERT_EQ(4, geometry.projection().minimalDimensions());

  for (int i = 0; i < 20; ++i) {
    Eigen::Vector3d p = geometry.createRandomVisiblePoint(2.0);
    Eigen::MatrixXd J, Jfd;
    geometry.euclideanToKeypointIntrinsicsJacobian(p, J, true, false, false);
    geometry.euclideanToKeypointIntrinsicsJacobianFiniteDifference(p, Jfd);  // d/d[xi fu fv cu cv]
    ASSERT_EQ(2, J.rows());
    ASSERT_EQ(4, J.cols());
    Eigen::MatrixXd Jref = Jfd.rightCols(4);
    ASSERT_DOUBLE_MX_EQ(J, Jref, 0.5, "");  // percent, as in the harness
  }

  // update() must leave xi alone and move fu fv cu cv.
  Eigen::MatrixXd P0, P1;
  geometry.projection().getParameters(P0);
  const double v[4] = {1.0, 2.0, 3.0, 4.0};
  geometry.projection().update(v);
  geometry.projection().getParameters(P1);
  EXPECT_EQ(P0(0), P1(0));
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(P0(i + 1) + v[i], P1(i + 1), 1e-12);
  }
}
