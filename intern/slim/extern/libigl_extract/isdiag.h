// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2016 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_ISDIAG_H
#define IGL_ISDIAG_H

#include <Eigen/Sparse>
namespace igl {
// Determine if a given matrix is diagonal: all non-zeros lie on the
// main diagonal.
//
// Inputs:
//   A  m by n sparse matrix
// Returns true iff and only if the matrix is diagonal.
template<typename Scalar> inline bool isdiag(const Eigen::SparseMatrix<Scalar> &A);
};  // namespace igl

#include "isdiag.cpp"

#endif
