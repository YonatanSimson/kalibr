#ifndef ASLAM_CAMERAS_RT4_DISTORTION_HPP
#define ASLAM_CAMERAS_RT4_DISTORTION_HPP

#include <Eigen/Dense>
#include <boost/serialization/nvp.hpp>
#include "StaticAssert.hpp"
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>

namespace sm {
class PropertyTree;
}  // namespace sm

namespace aslam {
namespace cameras {

/**
 * \class RadialTangential4Distortion
 * \brief RadialTangentialDistortion with the radial polynomial carried to r^8.
 *
 *   rad = k1 r^2 + k2 r^4 + k3 r^6 + k4 r^8,   r^2 = x^2 + y^2
 *   x' = x (1 + rad) + 2 p1 x y + p2 (r^2 + 2 x^2)
 *   y' = y (1 + rad) + 2 p2 x y + p1 (r^2 + 2 y^2)
 *
 * This is the lens model Insta360 cameras store per unit (MEI projection + k1..k4, p1, p2), so
 * an `omni-radtan4` calibration compares term by term with the factory record. Parameter order
 * is [k1 k2 p1 p2 k3 k4]: the first four are RadialTangentialDistortion's, so a radtan result
 * seeds this model unchanged. See OMNI_FIXED_XI.md.
 */
class RadialTangential4Distortion {
 public:

  enum {
    IntrinsicsDimension = 6
  };
  enum {
    DesignVariableDimension = IntrinsicsDimension
  };

  /// \brief The default constructor sets all values to zero.
  RadialTangential4Distortion();

  /// \brief A constructor that initializes all values.
  RadialTangential4Distortion(double k1, double k2, double p1, double p2, double k3, double k4);

  RadialTangential4Distortion(const sm::PropertyTree & config);

  virtual ~RadialTangential4Distortion();

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

  /// \brief Jacobian of the distortion wrt [k1 k2 p1 p2 k3 k4] at the undistorted point imageY.
  template<typename DERIVED_Y, typename DERIVED_JD>
  void distortParameterJacobian(
      const Eigen::MatrixBase<DERIVED_Y> & imageY,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  /// \brief aslam backend: update [k1 k2 p1 p2 k3 k4] += v.
  void update(const double * v);

  int minimalDimensions() const;

  void getParameters(Eigen::MatrixXd & P) const;

  void setParameters(const Eigen::MatrixXd & P);

  Eigen::Vector2i parameterSize() const;

  double k1() { return _k1; }
  double k2() { return _k2; }
  double p1() { return _p1; }
  double p2() { return _p2; }
  double k3() { return _k3; }
  double k4() { return _k4; }

  void clear() {
    _k1 = 0.0;
    _k2 = 0.0;
    _p1 = 0.0;
    _p2 = 0.0;
    _k3 = 0.0;
    _k4 = 0.0;
  }

  /// \brief Compatibility with boost::serialization.
  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };BOOST_SERIALIZATION_SPLIT_MEMBER();
  template<class Archive>
  void load(Archive & ar, const unsigned int version);
  template<class Archive>
  void save(Archive & ar, const unsigned int version) const;

  bool isBinaryEqual(const RadialTangential4Distortion & rhs) const;

  static RadialTangential4Distortion getTestDistortion();

  double _k1;
  double _k2;
  double _p1;
  double _p2;
  double _k3;
  double _k4;

 private:
  /// rad = k1 r^2 + k2 r^4 + k3 r^6 + k4 r^8 and dRad = d(rad)/d(r^2).
  void radial(double r2, double & rad, double & dRad) const;
};

}  // namespace cameras
}  // namespace aslam

#include "implementation/RadialTangential4Distortion.hpp"

SM_BOOST_CLASS_VERSION (aslam::cameras::RadialTangential4Distortion);

#endif /* ASLAM_CAMERAS_RT4_DISTORTION_HPP */
