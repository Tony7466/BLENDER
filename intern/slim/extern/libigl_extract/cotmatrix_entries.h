// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_COTMATRIX_ENTRIES_H
#define IGL_COTMATRIX_ENTRIES_H

#include <Eigen/Core>
namespace igl {
// COTMATRIX_ENTRIES compute the cotangents of each angle in mesh (V,F)
//
// Inputs:
//   V  #V by dim list of rest domain positions
//   F  #F by 3 list of triangle indices into V
// Outputs:
//     C  #F by 3 list of 1/2*cotangents corresponding angles
//       for triangles, columns correspond to edges [1,2],[2,0],[0,1]
//
template<typename DerivedV, typename DerivedF, typename DerivedC>
inline void cotmatrix_entries(const Eigen::PlainObjectBase<DerivedV> &V,
                              const Eigen::PlainObjectBase<DerivedF> &F,
                              Eigen::PlainObjectBase<DerivedC> &C);
}  // namespace igl

#include "cotmatrix_entries.cpp"

#endif
