#ifndef MATH_HPP
#define MATH_HPP

#include <iostream>
#include <cassert>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <glm/glm.hpp>

#define PI 3.141592653

/* Vec defines a column vector of size n */
template<int n, typename T = double> class Vec {
 private:
  T col[n];

 public:
  Vec()
  {
    for (int i = 0; i < n; i++) {
      col[i] = 0.0f;
    }
  }

  Vec(T x)
  {
    for (int i = 0; i < n; i++) {
      col[i] = x;
    }
  }

  Vec(T x, T y)
  {
    static_assert(n == 2, "Vec size must be 2");
    col[0] = x;
    col[1] = y;
  }

  Vec(T x, T y, T z)
  {
    static_assert(n == 3, "Vec size must be 3");
    col[0] = x;
    col[1] = y;
    col[2] = z;
  }

  Vec(glm::vec3 p)
  {
    static_assert(n == 3, "Vec size must be 3");
    col[0] = p.x;
    col[1] = p.y;
    col[2] = p.z;
  }

  Vec(T x, T y, T z, T w)
  {
    static_assert(n == 4, "Vec size must be 4");
    col[0] = x;
    col[1] = y;
    col[2] = z;
    col[3] = w;
  }

  T &operator[](int i)
  {
    assert(i >= 0 && i < n);
    return col[i];
  }

  const T &operator[](int i) const
  {
    assert(i >= 0 && i < n);
    return col[i];
  }
};

#define VecnT Vec<n, T>
#define nT template<int n, typename T = double>

nT VecnT operator+(const VecnT &u)
{
  return u;
}

nT VecnT operator+(const VecnT &u, const VecnT &v)
{
  VecnT w;
  for (int i = 0; i < n; i++) {
    w[i] = u[i] + v[i];
  }
  return w;
}

nT VecnT &operator+=(VecnT &u, const VecnT &v)
{
  return u = u + v;
}

nT VecnT operator-(const VecnT &u)
{
  VecnT v;
  for (int i = 0; i < n; i++) {
    v[i] = -u[i];
  }
  return v;
}

nT VecnT operator-(const VecnT &u, const VecnT &v)
{
  VecnT w;
  for (int i = 0; i < n; i++) {
    w[i] = u[i] - v[i];
  }
  return w;
}

nT VecnT &operator-=(VecnT &u, const VecnT &v)
{
  return u = u - v;
}

nT VecnT operator*(const T &a, const VecnT &u)
{
  VecnT v;
  for (int i = 0; i < n; i++) {
    v[i] = a * u[i];
  }
  return v;
}

nT VecnT operator*(const VecnT &u, const T &a)
{
  return a * u;
}

nT VecnT &operator*=(VecnT &u, const T &a)
{
  return u = u * a;
}

nT VecnT operator/(const VecnT &u, const T &a)
{
  return u * (1.0f / a);
}

nT VecnT &operator/=(VecnT &u, const T &a)
{
  return u = u / a;
}

nT bool operator==(const VecnT &u, const VecnT &v)
{
  for (int i = 0; i < n; i++) {
    if (u[i] != v[i]) {
      return false;
    }
  }
  return true;
}

nT bool operator!=(const VecnT &u, const VecnT &v)
{
  return !(u == v);
}

nT T dot(const VecnT &u, const VecnT &v)
{
  T d = 0.0f;
  for (int i = 0; i < n; i++) {
    d += u[i] * v[i];
  }
  return d;
}

nT T norm2(const VecnT &u)
{
  return dot(u, u);
}

nT T norm(const VecnT &u)
{
  return sqrt(norm2(u));
}

nT VecnT normalize(const VecnT &u)
{
  T m = norm(u);
  if (m == 0.0f) {
    return VecnT(0.0f);
  }
  return u / m;
}

template<typename T> Vec<3, T> cross(const Vec<3, T> &u, const Vec<3, T> &v)
{
  Vec<3, T> w;

  w[0] = u[1] * v[2] - u[2] * v[1];
  w[1] = u[2] * v[0] - u[0] * v[2];
  w[2] = u[0] * v[1] - u[1] * v[0];

  return w;
}

template<typename T> T stp(const Vec<3, T> &u, const Vec<3, T> &v, const Vec<3, T> &w)
{
  return dot(u, cross(v, w));
}

nT std::ostream &operator<<(std::ostream &out, const VecnT &u)
{
  out << "(";
  for (int i = 0; i < n; i++) {
    out << (i == 0 ? "" : ", ") << u[i];
  }
  out << ")";

  return out;
}

#undef VecnT
#undef nT

typedef Vec<2> Vec2;
typedef Vec<3> Vec3;
typedef Vec<4> Vec4;

#define VecnT Vec<n, T>
#define VecmT Vec<m, T>
#define MatmnT Mat<m, n, T>
#define MatnmT Mat<n, m, T>
#define MatnnT Mat<n, n, T>
#define mnT template<int m, int n, typename T = double>

/* Mat defines a matrix of size mxn, m rows, n colums */
template<int m, int n, typename T = double> class Mat {
 private:
  VecmT cols[n];

 public:
  Mat()
  {
    for (int i = 0; i < n; i++) {
      cols[i] = VecmT(0.0f);
    }
  }

  Mat(T x)
  {
    for (int i = 0; i < n; i++) {
      cols[i] = VecmT(0.0f);
      if (i < m) {
        cols[i][i] = x;
      }
    }
  }

  Mat(VecmT x, VecmT y)
  {
    static_assert(n == 2, "Matrix must have 2 columns");
    cols[0] = x;
    cols[1] = y;
  }

  Mat(VecmT x, VecmT y, VecmT z)
  {
    static_assert(n == 3, "Matrix must have 3 columns");
    cols[0] = x;
    cols[1] = y;
    cols[2] = z;
  }

  Mat(VecmT x, VecmT y, VecmT z, VecmT w)
  {
    static_assert(n == 4, "Matrix must have 4 columns");
    cols[0] = x;
    cols[1] = y;
    cols[2] = z;
    cols[3] = w;
  }

  static Mat rows(VecnT x, VecnT y)
  {
    Mat<2, n, T> M;

    for (int i = 0; i < n; i++) {
      M.col(i)[0] = x[i];
      M.col(i)[1] = y[i];
    }

    return M;
  }

  static Mat rows(VecnT x, VecnT y, VecnT z)
  {
    Mat<3, n, T> M;

    for (int i = 0; i < n; i++) {
      M.col(i)[0] = x[i];
      M.col(i)[1] = y[i];
      M.col(i)[2] = z[i];
    }

    return M;
  }

  static Mat rows(VecnT x, VecnT y, VecnT z, VecnT w)
  {
    Mat<4, n, T> M;

    for (int i = 0; i < n; i++) {
      M.col(i)[0] = x[i];
      M.col(i)[1] = y[i];
      M.col(i)[2] = z[i];
      M.col(i)[3] = w[i];
    }

    return M;
  }

  VecnT row(int i) const
  {
    VecnT R;

    for (int col = 0; col < n; col++) {
      R[col] = cols[col][i];
    }

    return R;
  }

  void set_row(int i, const VecnT &v)
  {
    for (int col = 0; col < n; ++col) {
      cols[col][i] = v[col];
    }
  }

  inline T &operator()(int i, int j)
  {
    return cols[j][i];
  }

  inline const T &operator()(int i, int j) const
  {
    return cols[j][i];
  }

  inline VecmT &col(int j)
  {
    return cols[j];
  }

  inline const VecmT &col(int j) const
  {
    return cols[j];
  }

  MatnmT t() const
  {
    return transpose(*this);
  }

  MatmnT inv() const
  {
    return inverse(*this);
  }
};

mnT MatmnT operator+(const MatmnT &A)
{
  return A;
}

mnT MatmnT operator-(const MatmnT &A)
{
  MatmnT B;
  for (int i = 0; i < n; i++) {
    B.col(i) = -A.col(i);
  }
  return B;
}

mnT MatmnT operator+(const MatmnT &A, const MatmnT &B)
{
  MatmnT &C;

  for (int i = 0; i < n; i++) {
    C.col(i) = A.col(i) + B.col(i);
  }

  return C;
}

mnT MatmnT &operator+=(MatmnT &A, const MatmnT &B)
{
  return A = A + B;
}

mnT MatmnT operator-(const MatmnT &A, const MatmnT &B)
{
  return A + (-B);
}

mnT MatmnT &operator-=(MatmnT &A, const MatmnT &B)
{
  return A = A - B;
}

mnT MatmnT operator*(const T &a, const MatmnT &A)
{
  MatmnT B;

  for (int j = 0; j < n; j++) {
    B.col(j) = a * A.col(j);
  }

  return B;
}

mnT MatmnT operator*(const MatmnT &A, const T &a)
{
  return a * A;
}

mnT MatmnT &operator*=(MatmnT &A, const T &a)
{
  return A = A * a;
}

mnT MatmnT operator/(const MatmnT &A, const T &a)
{
  return A * (1.0f / a);
}

mnT MatmnT &operator/=(MatmnT &A, const T &a)
{
  return A = A / a;
}

mnT VecmT operator*(const MatmnT &A, const VecnT &u)
{
  VecmT v = VecmT(0);

  for (int j = 0; j < n; j++) {
    v += A.col(j) * u[j];
  }

  return v;
}

template<int m, int n, int o, typename T>
Mat<m, o, T> operator*(const Mat<m, n, T> &A, const Mat<n, o, T> &B)
{
  Mat<m, o, T> C;

  for (int k = 0; k < o; k++) {
    C.col(k) = A * B.col(k);
  }

  return C;
}

mnT MatmnT *operator*=(const MatmnT &A, const MatnnT &B)
{
  return A = A * B;
}

mnT MatnmT transpose(const MatmnT &A)
{
  MatnmT B;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      B(j, i) = A(i, j);
    }
  }

  return B;
}

template<int n, typename T> VecnT diag(const MatnnT &A)
{
  VecnT u;

  for (int j = 0; j < n; j++) {
    u[j] = A(j, j);
  }

  return u;
}

template<int n, typename T> T trace(const MatnnT &A)
{
  T t = 0;

  for (int j = 0; j < n; j++) {
    t += A(j, j);
  }

  return t;
}

template<typename T> T det(const Mat<2, 2, T> &A)
{
  return A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0);
}

template<typename T> T det(const Mat<3, 3, T> &A)
{
  return stp(A.col(0), A.col(1), A.col(2));
}

template<typename T> Mat<2, 2, T> inverse(const Mat<2, 2, T> &A)
{
  return Mat<2, 2, T>(Vec<2, T>(A(1, 1), -A(1, 0)), Vec<2, T>(-A(0, 1), A(0, 0))) / det(A);
}

template<typename T> T wedge(const Vec<2, T> &u, const Vec<2, T> &v)
{
  return u[0] * v[1] - u[1] * v[0];
}

template<typename T> Mat<3, 3, T> inverse(const Mat<3, 3, T> &A)
{
  return Mat<3, 3, T>(
             cross(A.col(1), A.col(2)), cross(A.col(2), A.col(0)), cross(A.col(0), A.col(1)))
             .t() /
         det(A);
}

template<int n, typename T> MatnnT diag(const VecnT &u)
{
  MatnnT A = MatnnT(0);

  for (int j = 0; j < n; j++) {
    A(j, j) = u[j];
  }

  return A;
}

mnT MatmnT outer(const VecmT &u, const VecnT &v)
{
  MatmnT A;

  for (int j = 0; j < n; j++) {
    A.col(j) = u * v[j];
  }

  return A;
}

mnT T inner(const MatmnT &a, const MatmnT &b)
{
  T r = 0;

  for (int j = 0; j < n; j++) {
    for (int i = 0; i < m; i++) {
      r += a.col(j)[i] * b.col(j)[i];
    }
  }

  return r;
}

// Frobenius norm2
mnT T norm2_F(const MatmnT &A)
{
  T a = 0;

  for (int j = 0; j < n; j++) {
    a += norm2(A.col(j));
  }

  return a;
}

// Frobenius norm
mnT T norm_F(const MatmnT &A)
{
  return sqrt(norm2_F(A));
}

mnT std::ostream &operator<<(std::ostream &out, const MatmnT &A)
{
  MatnmT At = transpose(A);

  out << "(" << std::endl;
  for (int i = 0; i < m; i++) {
    out << "    " << At.col(i) << (i + 1 == m ? "" : ",") << std::endl;
  }
  out << ")";

  return out;
}

#undef VecnT
#undef VecmT
#undef MatmnT
#undef MatnmT
#undef MatnnT
#undef mnT

typedef Mat<2, 2> Mat2x2;
typedef Mat<3, 3> Mat3x3;
typedef Mat<3, 2> Mat3x2;
typedef Mat<2, 3> Mat2x3;

/* Eigen Declarations */
typedef Eigen::Matrix<double, 3, 3, 0, 3, 3> EigenMat3;
typedef Eigen::Matrix<double, 3, 1, 0, 3, 1> EigenVec3;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> EigenVecX;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> EigenMatX;
typedef Eigen::SparseMatrix<double> EigenSparseMatrix;
typedef Eigen::Triplet<double, int> EigenSparseMatrixTriplet;
#define block_vector(a) block<3, 1>(3 * (a), 0)

inline Vec3 eigenToVec3(const EigenVec3 &v)
{
  return Vec3(v[0], v[1], v[2]);
}

inline EigenVec3 vec3ToEigen(const Vec3 &v)
{
  return EigenVec3(v[0], v[1], v[2]);
}

inline glm::vec3 vec3ToGlmVec3(const Vec3 &v)
{
  return glm::vec3(v[0], v[1], v[2]);
}

inline Vec3 glmVec3ToVec3(const glm::vec3 &v)
{
  return Vec3(v[0], v[1], v[2]);
}

#endif
