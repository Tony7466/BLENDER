// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "find.h"

#include <iostream>

template<typename T, typename DerivedI, typename DerivedJ, typename DerivedV>
inline void igl::find(const Eigen::SparseMatrix<T> &X,
                      Eigen::MatrixBase<DerivedI> &I,
                      Eigen::MatrixBase<DerivedJ> &J,
                      Eigen::MatrixBase<DerivedV> &V)
{
  // Resize outputs to fit nonzeros
  I.derived().resize(X.nonZeros(), 1);
  J.derived().resize(X.nonZeros(), 1);
  V.derived().resize(X.nonZeros(), 1);

  int i = 0;
  // Iterate over outside
  for (int k = 0; k < X.outerSize(); ++k) {
    // Iterate over inside
    for (typename Eigen::SparseMatrix<T>::InnerIterator it(X, k); it; ++it) {
      V(i) = it.value();
      I(i) = it.row();
      J(i) = it.col();
      i++;
    }
  }
}

template<typename DerivedX, typename DerivedI, typename DerivedJ, typename DerivedV>
inline void igl::find(const Eigen::PlainObjectBase<DerivedX> &X,
                      Eigen::PlainObjectBase<DerivedI> &I,
                      Eigen::PlainObjectBase<DerivedJ> &J,
                      Eigen::PlainObjectBase<DerivedV> &V)
{
  const int nnz = X.count();
  I.resize(nnz, 1);
  J.resize(nnz, 1);
  V.resize(nnz, 1);
  {
    int k = 0;
    for (int j = 0; j < X.cols(); j++) {
      for (int i = 0; i < X.rows(); i++) {
        if (X(i, j)) {
          I(k) = i;
          J(k) = j;
          V(k) = X(i, j);
          k++;
        }
      }
    }
  }
}

template<typename DerivedX, typename DerivedI>
inline void igl::find(const Eigen::PlainObjectBase<DerivedX> &X,
                      Eigen::PlainObjectBase<DerivedI> &I)
{
  const int nnz = X.count();
  I.resize(nnz, 1);
  {
    int k = 0;
    for (int j = 0; j < X.cols(); j++) {
      for (int i = 0; i < X.rows(); i++) {
        if (X(i, j)) {
          I(k) = i + X.rows() * j;
          k++;
        }
      }
    }
  }
}

template<typename T>
inline void igl::find(const Eigen::SparseVector<T> &X,
                      Eigen::Matrix<int, Eigen::Dynamic, 1> &I,
                      Eigen::Matrix<T, Eigen::Dynamic, 1> &V)
{
  // Resize outputs to fit nonzeros
  I.resize(X.nonZeros());
  V.resize(X.nonZeros());

  int i = 0;
  // loop over non-zeros
  for (typename Eigen::SparseVector<T>::InnerIterator it(X); it; ++it) {
    I(i) = it.index();
    V(i) = it.value();
    i++;
  }
}
