// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "edge_lengths.h"
#include "parallel_for.h"

template<typename DerivedV, typename DerivedF, typename DerivedL>
inline void igl::edge_lengths(const Eigen::PlainObjectBase<DerivedV> &V,
                              const Eigen::PlainObjectBase<DerivedF> &F,
                              Eigen::PlainObjectBase<DerivedL> &L)
{
  igl::squared_edge_lengths(V, F, L);
  L = L.array().sqrt().eval();
}

template<typename DerivedV, typename DerivedF, typename DerivedL>
inline void igl::squared_edge_lengths(const Eigen::PlainObjectBase<DerivedV> &V,
                                      const Eigen::PlainObjectBase<DerivedF> &F,
                                      Eigen::PlainObjectBase<DerivedL> &L)
{
  using namespace std;
  const int m = F.rows();
  assert(F.cols() == 3);

  L.resize(m, 3);
  // loop over faces
  parallel_for(
      m,
      [&V, &F, &L](const int i) {
        L(i, 0) = (V.row(F(i, 1)) - V.row(F(i, 2))).squaredNorm();
        L(i, 1) = (V.row(F(i, 2)) - V.row(F(i, 0))).squaredNorm();
        L(i, 2) = (V.row(F(i, 0)) - V.row(F(i, 1))).squaredNorm();
      },
      1000);
}
