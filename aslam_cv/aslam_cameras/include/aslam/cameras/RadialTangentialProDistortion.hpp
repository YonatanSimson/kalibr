#ifndef ASLAM_CAMERAS_RTPRO_DISTORTION_HPP
#define ASLAM_CAMERAS_RTPRO_DISTORTION_HPP

#include <Eigen/Dense>
#include <boost/serialization/nvp.hpp>
#include "StaticAssert.hpp"
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>
#include <array>
#include <vector>

namespace sm {
class PropertyTree;
}  // namespace sm

namespace aslam {
namespace cameras {

/**
 * \class RadialTangentialProDistortion
 * \brief Insta360's 13-term `offset_v6` distortion (telemetry-parser `MeiDistortion::RadtanPro`).
 *
 *   radial = 1 + r^2 (d0 + r^2 (d1 + r^2 (d2 + r^2 (d3 + r^2 d4))))
 *   a = d5 + d7 r^2,   b = d6 + d8 r^2
 *   x' = x radial + (r^2 + 2 x^2) a + 2 x y b + d9 r^2 + d11 r^4
 *   y' = y radial + (r^2 + 2 y^2) b + 2 x y a + d10 r^2 + d12 r^4
 *
 * Parameters are d0..d12 in the vendor's order, so a factory record seeds this model unchanged.
 * In radtan order `a` is p2 and `b` is p1. See OMNI_PATCH.md.
 *
 * KALIBR_RADTANPRO_FIXED="i j ..." holds the listed coefficients at their current value: they
 * leave the parameter block, as fixed xi does (radtanpro::setFixed overrides it in-process).
 */
class RadialTangentialProDistortion {
 public:

  enum {
    IntrinsicsDimension = 13
  };
  enum {
    DesignVariableDimension = IntrinsicsDimension
  };

  /// \brief The default constructor sets all values to zero.
  RadialTangentialProDistortion();

  /// \brief d0..d12 in the vendor's order.
  explicit RadialTangentialProDistortion(const std::vector<double> & d);

  RadialTangentialProDistortion(const sm::PropertyTree & config);

  virtual ~RadialTangentialProDistortion();

  /// \brief Apply distortion to a point in the normalized image plane (in place).
  template<typename DERIVED_Y>
  void distort(const Eigen::MatrixBase<DERIVED_Y> & y) const;

  /// \brief Apply distortion; outJy is the Jacobian wrt the input point.
  template<typename DERIVED_Y, typename DERIVED_JY>
  void distort(const Eigen::MatrixBase<DERIVED_Y> & y,
               const Eigen::MatrixBase<DERIVED_JY> & outJy) const;

  /// \brief Undistort (iterative Gauss-Newton on distort).
  template<typename DERIVED>
  void undistort(const Eigen::MatrixBase<DERIVED> & y) const;

  /// \brief Undistort; outJy is the Jacobian of the undistortion wrt the input point.
  template<typename DERIVED, typename DERIVED_JY>
  void undistort(const Eigen::MatrixBase<DERIVED> & y,
                 const Eigen::MatrixBase<DERIVED_JY> & outJy) const;

  /// \brief Jacobian of the distortion wrt the free coefficients (d0..d12 minus the fixed ones,
  /// in order) at the undistorted point imageY. 2 x minimalDimensions().
  template<typename DERIVED_Y, typename DERIVED_JD>
  void distortParameterJacobian(
      const Eigen::MatrixBase<DERIVED_Y> & imageY,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  /// \brief aslam backend: the free coefficients += v.
  void update(const double * v);

  /// \brief 13 minus the number of fixed coefficients.
  int minimalDimensions() const;

  /// \brief All 13 coefficients, fixed ones included.
  void getParameters(Eigen::MatrixXd & P) const;

  void setParameters(const Eigen::MatrixXd & P);

  Eigen::Vector2i parameterSize() const;

  double d(int i) const { return _d.at(i); }

  void clear() { _d.fill(0.0); }

  /// \brief Compatibility with boost::serialization.
  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };BOOST_SERIALIZATION_SPLIT_MEMBER();
  template<class Archive>
  void load(Archive & ar, const unsigned int version);
  template<class Archive>
  void save(Archive & ar, const unsigned int version) const;

  bool isBinaryEqual(const RadialTangentialProDistortion & rhs) const;

  static RadialTangentialProDistortion getTestDistortion();

  std::array<double, 13> _d;

 private:
  /// The distortion and, when J is given, its full 2 x 13 parameter Jacobian.
  void distortWithParameterJacobian(double x, double y, double & xd, double & yd,
                                    Eigen::Matrix<double, 2, 13> * J) const;
};

namespace radtanpro {
/// The coefficients held fixed: KALIBR_RADTANPRO_FIXED, unless setFixed overrode it.
const std::vector<int> & fixed();
/// Override KALIBR_RADTANPRO_FIXED for this process (tests). Set it before building an
/// optimisation problem: the parameter block size must not change under one.
void setFixed(const std::vector<int> & indices);
/// True when coefficient i is held fixed.
bool isFixed(int i);
}  // namespace radtanpro

}  // namespace cameras
}  // namespace aslam

#include "implementation/RadialTangentialProDistortion.hpp"

SM_BOOST_CLASS_VERSION (aslam::cameras::RadialTangentialProDistortion);

#endif /* ASLAM_CAMERAS_RTPRO_DISTORTION_HPP */
