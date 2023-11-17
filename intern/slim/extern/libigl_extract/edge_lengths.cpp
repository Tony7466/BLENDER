// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "edge_lengths.h"
#include "squared_edge_lengths.h"

template<typename DerivedV, typename DerivedF, typename DerivedL>
IGL_INLINE void igl::edge_lengths(const Eigen::PlainObjectBase<DerivedV> &V,
                                  const Eigen::PlainObjectBase<DerivedF> &F,
                                  Eigen::PlainObjectBase<DerivedL> &L)
{
  igl::squared_edge_lengths(V, F, L);
  L = L.array().sqrt().eval();
}
