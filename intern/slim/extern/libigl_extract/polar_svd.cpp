// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "polar_svd.h"
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <iostream>

// Adapted from Olga's CGAL mentee's ARAP code
template<typename DerivedA, typename DerivedR, typename DerivedT>
inline void igl::polar_svd(const Eigen::PlainObjectBase<DerivedA> &A,
                           Eigen::PlainObjectBase<DerivedR> &R,
                           Eigen::PlainObjectBase<DerivedT> &T)
{
  DerivedA U;
  DerivedA V;
  Eigen::Matrix<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime, 1> S;
  return igl::polar_svd(A, R, T, U, S, V);
}

template<typename DerivedA,
         typename DerivedR,
         typename DerivedT,
         typename DerivedU,
         typename DerivedS,
         typename DerivedV>
inline void igl::polar_svd(const Eigen::PlainObjectBase<DerivedA> &A,
                           Eigen::PlainObjectBase<DerivedR> &R,
                           Eigen::PlainObjectBase<DerivedT> &T,
                           Eigen::PlainObjectBase<DerivedU> &U,
                           Eigen::PlainObjectBase<DerivedS> &S,
                           Eigen::PlainObjectBase<DerivedV> &V)
{
  using namespace std;
  Eigen::JacobiSVD<DerivedA> svd;
  svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  V = svd.matrixV();
  S = svd.singularValues();
  R = U * V.transpose();
  const auto &SVT = S.asDiagonal() * V.adjoint();
  // Check for reflection
  if (R.determinant() < 0) {
    // Annoyingly the .eval() is necessary
    auto W = V.eval();
    W.col(V.cols() - 1) *= -1.;
    R = U * W.transpose();
    T = W * SVT;
  }
  else {
    T = V * SVT;
  }
}
