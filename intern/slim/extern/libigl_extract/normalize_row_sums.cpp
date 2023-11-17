// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "normalize_row_sums.h"

template<typename DerivedA, typename DerivedB>
IGL_INLINE void igl::normalize_row_sums(const Eigen::MatrixBase<DerivedA> &A,
                                        Eigen::MatrixBase<DerivedB> &B)
{
#ifndef NDEBUG
  // loop over rows
  for (int i = 0; i < A.rows(); i++) {
    typename DerivedB::Scalar sum = A.row(i).sum();
    assert(sum != 0);
  }
#endif
  B = (A.array().colwise() / A.rowwise().sum().array()).eval();
}
