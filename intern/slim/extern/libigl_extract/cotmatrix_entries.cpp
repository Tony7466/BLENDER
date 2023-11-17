// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "cotmatrix_entries.h"
#include "doublearea.h"
#include "edge_lengths.h"
#include "squared_edge_lengths.h"

template<typename DerivedV, typename DerivedF, typename DerivedC>
inline void igl::cotmatrix_entries(const Eigen::PlainObjectBase<DerivedV> &V,
                                   const Eigen::PlainObjectBase<DerivedF> &F,
                                   Eigen::PlainObjectBase<DerivedC> &C)
{
  using namespace std;
  using namespace Eigen;
  // Number of elements
  int m = F.rows();

  assert(F.cols() == 3);

  // Law of cosines + law of sines
  // Compute Squared Edge lenghts
  Matrix<typename DerivedC::Scalar, Dynamic, 3> l2;
  igl::squared_edge_lengths(V, F, l2);
  // Compute Edge lenghts
  Matrix<typename DerivedC::Scalar, Dynamic, 3> l;
  l = l2.array().sqrt();

  // double area
  Matrix<typename DerivedC::Scalar, Dynamic, 1> dblA;
  doublearea(l, dblA);
  // cotangents and diagonal entries for element matrices
  // correctly divided by 4 (alec 2010)
  C.resize(m, 3);
  for (int i = 0; i < m; i++) {
    C(i, 0) = (l2(i, 1) + l2(i, 2) - l2(i, 0)) / dblA(i) / 4.0;
    C(i, 1) = (l2(i, 2) + l2(i, 0) - l2(i, 1)) / dblA(i) / 4.0;
    C(i, 2) = (l2(i, 0) + l2(i, 1) - l2(i, 2)) / dblA(i) / 4.0;
  }
}
