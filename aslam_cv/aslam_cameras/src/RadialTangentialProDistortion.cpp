#include <aslam/cameras/RadialTangentialProDistortion.hpp>
#include <sm/PropertyTree.hpp>
#include <sm/serialization_macros.hpp>
#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <stdexcept>

namespace aslam {
namespace cameras {

namespace radtanpro {
namespace {
std::vector<int> & fixedStorage() {
  static std::vector<int> value = [] {
    std::vector<int> v;
    const char * s = std::getenv("KALIBR_RADTANPRO_FIXED");
    if (s != nullptr) {
      std::istringstream is(s);
      int i;
      while (is >> i) {
        if (i < 0 || i > 12) {
          throw std::runtime_error("KALIBR_RADTANPRO_FIXED: coefficient index out of 0..12");
        }
        v.push_back(i);
      }
    }
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
  }();
  return value;
}
}  // namespace

const std::vector<int> & fixed() {
  return fixedStorage();
}

void setFixed(const std::vector<int> & indices) {
  std::vector<int> v(indices);
  for (int i : v) {
    if (i < 0 || i > 12) {
      throw std::runtime_error("radtanpro::setFixed: coefficient index out of 0..12");
    }
  }
  std::sort(v.begin(), v.end());
  v.erase(std::unique(v.begin(), v.end()), v.end());
  fixedStorage() = v;
}

bool isFixed(int i) {
  const std::vector<int> & v = fixedStorage();
  return std::binary_search(v.begin(), v.end(), i);
}
}  // namespace radtanpro

RadialTangentialProDistortion::RadialTangentialProDistortion() {
  _d.fill(0.0);
}

RadialTangentialProDistortion::RadialTangentialProDistortion(const std::vector<double> & d) {
  if (d.size() != 13) {
    throw std::runtime_error("RadialTangentialProDistortion takes 13 coefficients d0..d12");
  }
  std::copy(d.begin(), d.end(), _d.begin());
}

RadialTangentialProDistortion::RadialTangentialProDistortion(
    const sm::PropertyTree & config) {
  for (int i = 0; i < 13; ++i) {
    _d[i] = config.getDouble("d" + std::to_string(i));
  }
}

RadialTangentialProDistortion::~RadialTangentialProDistortion() {
}

// aslam::backend compatibility
void RadialTangentialProDistortion::update(const double * v) {
  int k = 0;
  for (int i = 0; i < 13; ++i) {
    if (!radtanpro::isFixed(i)) {
      _d[i] += v[k++];
    }
  }
}

int RadialTangentialProDistortion::minimalDimensions() const {
  return IntrinsicsDimension - static_cast<int>(radtanpro::fixed().size());
}

void RadialTangentialProDistortion::getParameters(Eigen::MatrixXd & S) const {
  S.resize(13, 1);
  for (int i = 0; i < 13; ++i) {
    S(i, 0) = _d[i];
  }
}

void RadialTangentialProDistortion::setParameters(const Eigen::MatrixXd & S) {
  for (int i = 0; i < 13; ++i) {
    _d[i] = S(i, 0);
  }
}

bool RadialTangentialProDistortion::isBinaryEqual(
    const RadialTangentialProDistortion & rhs) const {
  return _d == rhs._d;
}

Eigen::Vector2i RadialTangentialProDistortion::parameterSize() const {
  return Eigen::Vector2i(13, 1);
}

RadialTangentialProDistortion RadialTangentialProDistortion::getTestDistortion() {
  // Mild radial terms with small decentering and prism, so the harness's generic test geometry
  // stays invertible over the whole image.
  return RadialTangentialProDistortion(std::vector<double>{
      -0.2, 0.13, -0.02, 0.005, 0.001, 0.0005, 0.0004, 0.0003, -0.0002, 0.0002, -0.0001, 0.0001,
      -0.00005});
}

}  // namespace cameras
}  // namespace aslam
