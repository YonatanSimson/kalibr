#include <aslam/cameras/RadialTangential4Distortion.hpp>
#include <sm/PropertyTree.hpp>
#include <sm/serialization_macros.hpp>

namespace aslam {
namespace cameras {

RadialTangential4Distortion::RadialTangential4Distortion()
    : _k1(0),
      _k2(0),
      _p1(0),
      _p2(0),
      _k3(0),
      _k4(0) {
}

RadialTangential4Distortion::RadialTangential4Distortion(double k1, double k2,
                                                         double p1, double p2,
                                                         double k3, double k4)
    : _k1(k1),
      _k2(k2),
      _p1(p1),
      _p2(p2),
      _k3(k3),
      _k4(k4) {
}

RadialTangential4Distortion::RadialTangential4Distortion(
    const sm::PropertyTree & config) {
  _k1 = config.getDouble("k1");
  _k2 = config.getDouble("k2");
  _p1 = config.getDouble("p1");
  _p2 = config.getDouble("p2");
  _k3 = config.getDouble("k3");
  _k4 = config.getDouble("k4");
}

RadialTangential4Distortion::~RadialTangential4Distortion() {
}

// aslam::backend compatibility
void RadialTangential4Distortion::update(const double * v) {
  _k1 += v[0];
  _k2 += v[1];
  _p1 += v[2];
  _p2 += v[3];
  _k3 += v[4];
  _k4 += v[5];
}

int RadialTangential4Distortion::minimalDimensions() const {
  return IntrinsicsDimension;
}

void RadialTangential4Distortion::getParameters(Eigen::MatrixXd & S) const {
  S.resize(6, 1);
  S << _k1, _k2, _p1, _p2, _k3, _k4;
}

void RadialTangential4Distortion::setParameters(const Eigen::MatrixXd & S) {
  _k1 = S(0, 0);
  _k2 = S(1, 0);
  _p1 = S(2, 0);
  _p2 = S(3, 0);
  _k3 = S(4, 0);
  _k4 = S(5, 0);
}

bool RadialTangential4Distortion::isBinaryEqual(
    const RadialTangential4Distortion & rhs) const {
  return SM_CHECKMEMBERSSAME(rhs, _k1) && SM_CHECKMEMBERSSAME(rhs, _k2)
      && SM_CHECKMEMBERSSAME(rhs, _p1) && SM_CHECKMEMBERSSAME(rhs, _p2)
      && SM_CHECKMEMBERSSAME(rhs, _k3) && SM_CHECKMEMBERSSAME(rhs, _k4);
}

Eigen::Vector2i RadialTangential4Distortion::parameterSize() const {
  return Eigen::Vector2i(6, 1);
}

RadialTangential4Distortion RadialTangential4Distortion::getTestDistortion() {
  return RadialTangential4Distortion(-0.2, 0.13, 0.0005, 0.0005, -0.02, 0.005);
}

}  // namespace cameras
}  // namespace aslam
