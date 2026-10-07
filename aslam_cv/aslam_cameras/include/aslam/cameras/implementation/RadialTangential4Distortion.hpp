namespace aslam {
namespace cameras {

inline void RadialTangential4Distortion::radial(double r2, double & rad, double & dRad) const {
  rad = r2 * (_k1 + r2 * (_k2 + r2 * (_k3 + r2 * _k4)));
  dRad = _k1 + r2 * (2.0 * _k2 + r2 * (3.0 * _k3 + r2 * 4.0 * _k4));
}

template<typename DERIVED_Y>
void RadialTangential4Distortion::distort(
    const Eigen::MatrixBase<DERIVED_Y> & yconst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);

  Eigen::MatrixBase<DERIVED_Y> & y =
      const_cast<Eigen::MatrixBase<DERIVED_Y> &>(yconst);
  y.derived().resize(2);

  const double mx2 = y[0] * y[0];
  const double my2 = y[1] * y[1];
  const double mxy = y[0] * y[1];
  const double r2 = mx2 + my2;
  double rad, dRad;
  radial(r2, rad, dRad);
  y[0] += y[0] * rad + 2.0 * _p1 * mxy + _p2 * (r2 + 2.0 * mx2);
  y[1] += y[1] * rad + 2.0 * _p2 * mxy + _p1 * (r2 + 2.0 * my2);
}

template<typename DERIVED_Y, typename DERIVED_JY>
void RadialTangential4Distortion::distort(
    const Eigen::MatrixBase<DERIVED_Y> & yconst,
    const Eigen::MatrixBase<DERIVED_JY> & outJy) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JY>, 2, 2);

  Eigen::MatrixBase<DERIVED_JY> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JY> &>(outJy);
  J.derived().resize(2, 2);
  J.setZero();

  Eigen::MatrixBase<DERIVED_Y> & y =
      const_cast<Eigen::MatrixBase<DERIVED_Y> &>(yconst);
  y.derived().resize(2);

  const double mx2 = y[0] * y[0];
  const double my2 = y[1] * y[1];
  const double mxy = y[0] * y[1];
  const double r2 = mx2 + my2;
  double rad, dRad;
  radial(r2, rad, dRad);

  // d(rad)/dx = 2 x dRad, d(rad)/dy = 2 y dRad
  J(0, 0) = 1.0 + rad + 2.0 * mx2 * dRad + 2.0 * _p1 * y[1] + 6.0 * _p2 * y[0];
  J(0, 1) = 2.0 * mxy * dRad + 2.0 * _p1 * y[0] + 2.0 * _p2 * y[1];
  J(1, 0) = J(0, 1);
  J(1, 1) = 1.0 + rad + 2.0 * my2 * dRad + 6.0 * _p1 * y[1] + 2.0 * _p2 * y[0];

  y[0] += y[0] * rad + 2.0 * _p1 * mxy + _p2 * (r2 + 2.0 * mx2);
  y[1] += y[1] * rad + 2.0 * _p2 * mxy + _p1 * (r2 + 2.0 * my2);
}

template<typename DERIVED>
void RadialTangential4Distortion::undistort(
    const Eigen::MatrixBase<DERIVED> & yconst) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED>, 2);

  Eigen::MatrixBase<DERIVED> & y =
      const_cast<Eigen::MatrixBase<DERIVED> &>(yconst);
  y.derived().resize(2);

  // Gauss-Newton on distort(ybar) = y. The r^8 term makes the function steeper than radtan's,
  // so allow more iterations than RadialTangentialDistortion's 5.
  Eigen::Vector2d ybar = y;
  const int n = 20;
  Eigen::Matrix2d F;
  Eigen::Vector2d y_tmp;

  for (int i = 0; i < n; i++) {
    y_tmp = ybar;
    distort(y_tmp, F);
    Eigen::Vector2d e(y - y_tmp);
    Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;
    ybar += du;
    if (e.dot(e) < 1e-15)
      break;
  }
  y = ybar;
}

template<typename DERIVED, typename DERIVED_JY>
void RadialTangential4Distortion::undistort(
    const Eigen::MatrixBase<DERIVED> & yconst,
    const Eigen::MatrixBase<DERIVED_JY> & outJy) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JY>, 2, 2);

  Eigen::MatrixBase<DERIVED> & y =
      const_cast<Eigen::MatrixBase<DERIVED> &>(yconst);
  y.derived().resize(2);

  // (f^-1)' = (f'(f^-1(y)))^-1
  undistort(y);
  Eigen::Vector2d kp = y;
  Eigen::Matrix2d Jd;
  distort(kp, Jd);

  DERIVED_JY & J = const_cast<DERIVED_JY &>(outJy.derived());
  J = Jd.inverse();
}

template<typename DERIVED_Y, typename DERIVED_JD>
void RadialTangential4Distortion::distortParameterJacobian(
    const Eigen::MatrixBase<DERIVED_Y> & imageY,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JD>, 2, 6);

  const double y0 = imageY[0];
  const double y1 = imageY[1];
  const double r2 = y0 * y0 + y1 * y1;
  const double r4 = r2 * r2;
  const double r6 = r4 * r2;
  const double r8 = r4 * r4;

  Eigen::MatrixBase<DERIVED_JD> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd);
  J.derived().resize(2, 6);
  J.setZero();

  // columns: k1 k2 p1 p2 k3 k4
  J(0, 0) = y0 * r2;
  J(0, 1) = y0 * r4;
  J(0, 2) = 2.0 * y0 * y1;
  J(0, 3) = r2 + 2.0 * y0 * y0;
  J(0, 4) = y0 * r6;
  J(0, 5) = y0 * r8;

  J(1, 0) = y1 * r2;
  J(1, 1) = y1 * r4;
  J(1, 2) = r2 + 2.0 * y1 * y1;
  J(1, 3) = 2.0 * y0 * y1;
  J(1, 4) = y1 * r6;
  J(1, 5) = y1 * r8;
}

template<class Archive>
void RadialTangential4Distortion::save(Archive & ar,
                                       const unsigned int /* version */) const {
  ar << BOOST_SERIALIZATION_NVP(_k1);
  ar << BOOST_SERIALIZATION_NVP(_k2);
  ar << BOOST_SERIALIZATION_NVP(_p1);
  ar << BOOST_SERIALIZATION_NVP(_p2);
  ar << BOOST_SERIALIZATION_NVP(_k3);
  ar << BOOST_SERIALIZATION_NVP(_k4);
}

template<class Archive>
void RadialTangential4Distortion::load(Archive & ar,
                                       const unsigned int version) {
  SM_ASSERT_LE(std::runtime_error, version,
               (unsigned int) CLASS_SERIALIZATION_VERSION,
               "Unsupported serialization version");

  ar >> BOOST_SERIALIZATION_NVP(_k1);
  ar >> BOOST_SERIALIZATION_NVP(_k2);
  ar >> BOOST_SERIALIZATION_NVP(_p1);
  ar >> BOOST_SERIALIZATION_NVP(_p2);
  ar >> BOOST_SERIALIZATION_NVP(_k3);
  ar >> BOOST_SERIALIZATION_NVP(_k4);
}

}  // namespace cameras
}  // namespace aslam
