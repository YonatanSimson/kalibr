namespace aslam {
namespace cameras {

inline void RadialTangentialProDistortion::distortWithParameterJacobian(
    double x, double y, double & xd, double & yd, Eigen::Matrix<double, 2, 13> * J) const {
  const std::array<double, 13> & d = _d;
  const double r2 = x * x + y * y;
  const double r4 = r2 * r2;
  const double radial = 1.0 + r2 * (d[0] + r2 * (d[1] + r2 * (d[2] + r2 * (d[3] + r2 * d[4]))));
  const double a = d[5] + d[7] * r2;
  const double b = d[6] + d[8] * r2;
  const double tx = r2 + 2.0 * x * x;  // multiplies a on x
  const double ty = r2 + 2.0 * y * y;  // multiplies b on y
  const double xy2 = 2.0 * x * y;
  xd = x * radial + tx * a + xy2 * b + d[9] * r2 + d[11] * r4;
  yd = y * radial + ty * b + xy2 * a + d[10] * r2 + d[12] * r4;
  if (J == nullptr) {
    return;
  }
  J->setZero();
  double rk = r2;  // r^2, r^4, ..., r^10 for d0..d4
  for (int k = 0; k < 5; ++k) {
    (*J)(0, k) = x * rk;
    (*J)(1, k) = y * rk;
    rk *= r2;
  }
  (*J)(0, 5) = tx;
  (*J)(1, 5) = xy2;
  (*J)(0, 6) = xy2;
  (*J)(1, 6) = ty;
  (*J)(0, 7) = tx * r2;
  (*J)(1, 7) = xy2 * r2;
  (*J)(0, 8) = xy2 * r2;
  (*J)(1, 8) = ty * r2;
  (*J)(0, 9) = r2;
  (*J)(1, 10) = r2;
  (*J)(0, 11) = r4;
  (*J)(1, 12) = r4;
}

template<typename DERIVED_Y>
void RadialTangentialProDistortion::distort(
    const Eigen::MatrixBase<DERIVED_Y> & yconst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);

  Eigen::MatrixBase<DERIVED_Y> & y =
      const_cast<Eigen::MatrixBase<DERIVED_Y> &>(yconst);
  y.derived().resize(2);

  double xd, yd;
  distortWithParameterJacobian(y[0], y[1], xd, yd, nullptr);
  y[0] = xd;
  y[1] = yd;
}

template<typename DERIVED_Y, typename DERIVED_JY>
void RadialTangentialProDistortion::distort(
    const Eigen::MatrixBase<DERIVED_Y> & yconst,
    const Eigen::MatrixBase<DERIVED_JY> & outJy) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JY>, 2, 2);

  Eigen::MatrixBase<DERIVED_JY> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JY> &>(outJy);
  J.derived().resize(2, 2);

  Eigen::MatrixBase<DERIVED_Y> & y =
      const_cast<Eigen::MatrixBase<DERIVED_Y> &>(yconst);
  y.derived().resize(2);

  const std::array<double, 13> & d = _d;
  const double x = y[0];
  const double v = y[1];
  const double r2 = x * x + v * v;
  const double radial = 1.0 + r2 * (d[0] + r2 * (d[1] + r2 * (d[2] + r2 * (d[3] + r2 * d[4]))));
  // d(radial)/d(r^2)
  const double dRadial = d[0] + r2 * (2.0 * d[1] + r2 * (3.0 * d[2] + r2 * (4.0 * d[3] + r2 * 5.0 * d[4])));
  const double a = d[5] + d[7] * r2;
  const double b = d[6] + d[8] * r2;
  const double tx = r2 + 2.0 * x * x;
  const double ty = r2 + 2.0 * v * v;

  // d(r^2)/dx = 2x, d(r^2)/dy = 2y; a and b depend on r^2 through d7, d8.
  J(0, 0) = radial + 2.0 * x * x * dRadial + 6.0 * x * a + 2.0 * x * d[7] * tx + 2.0 * v * b
      + 4.0 * x * x * v * d[8] + 2.0 * x * d[9] + 4.0 * x * r2 * d[11];
  J(0, 1) = 2.0 * x * v * dRadial + 2.0 * v * a + 2.0 * v * d[7] * tx + 2.0 * x * b
      + 4.0 * x * v * v * d[8] + 2.0 * v * d[9] + 4.0 * v * r2 * d[11];
  J(1, 0) = 2.0 * x * v * dRadial + 2.0 * x * b + 2.0 * x * d[8] * ty + 2.0 * v * a
      + 4.0 * x * x * v * d[7] + 2.0 * x * d[10] + 4.0 * x * r2 * d[12];
  J(1, 1) = radial + 2.0 * v * v * dRadial + 6.0 * v * b + 2.0 * v * d[8] * ty + 2.0 * x * a
      + 4.0 * x * v * v * d[7] + 2.0 * v * d[10] + 4.0 * v * r2 * d[12];

  double xd, yd;
  distortWithParameterJacobian(x, v, xd, yd, nullptr);
  y[0] = xd;
  y[1] = yd;
}

template<typename DERIVED>
void RadialTangentialProDistortion::undistort(
    const Eigen::MatrixBase<DERIVED> & yconst) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED>, 2);

  Eigen::MatrixBase<DERIVED> & y =
      const_cast<Eigen::MatrixBase<DERIVED> &>(yconst);
  y.derived().resize(2);

  // Gauss-Newton on distort(ybar) = y, as RadialTangential4Distortion: the r^8 / r^10 terms make
  // the map steep at the rim, so allow more iterations than radtan's 5.
  Eigen::Vector2d ybar = y;
  const int n = 30;
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
void RadialTangentialProDistortion::undistort(
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
void RadialTangentialProDistortion::distortParameterJacobian(
    const Eigen::MatrixBase<DERIVED_Y> & imageY,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);

  Eigen::Matrix<double, 2, 13> Jfull;
  double xd, yd;
  distortWithParameterJacobian(imageY[0], imageY[1], xd, yd, &Jfull);

  Eigen::MatrixBase<DERIVED_JD> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd);
  J.derived().resize(2, minimalDimensions());
  int col = 0;
  for (int i = 0; i < 13; ++i) {
    if (!radtanpro::isFixed(i)) {
      J.col(col++) = Jfull.col(i);
    }
  }
}

template<class Archive>
void RadialTangentialProDistortion::save(Archive & ar,
                                         const unsigned int /* version */) const {
  static const char * names[13] = {"d0", "d1", "d2", "d3", "d4", "d5", "d6",
                                   "d7", "d8", "d9", "d10", "d11", "d12"};
  for (int i = 0; i < 13; ++i) {
    ar << boost::serialization::make_nvp(names[i], _d[i]);
  }
}

template<class Archive>
void RadialTangentialProDistortion::load(Archive & ar,
                                         const unsigned int version) {
  SM_ASSERT_LE(std::runtime_error, version,
               (unsigned int) CLASS_SERIALIZATION_VERSION,
               "Unsupported serialization version");
  static const char * names[13] = {"d0", "d1", "d2", "d3", "d4", "d5", "d6",
                                   "d7", "d8", "d9", "d10", "d11", "d12"};
  for (int i = 0; i < 13; ++i) {
    ar >> boost::serialization::make_nvp(names[i], _d[i]);
  }
}

}  // namespace cameras
}  // namespace aslam
