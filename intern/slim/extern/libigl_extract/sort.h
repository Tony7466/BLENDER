// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_SORT_H
#define IGL_SORT_H

#include <Eigen/Core>
#include <vector>
namespace igl {

// Sort the elements of a matrix X along a given dimension like matlabs sort
// function
//
// Templates:
//   DerivedX derived scalar type, e.g. MatrixXi or MatrixXd
//   DerivedIX derived integer type, e.g. MatrixXi
// Inputs:
//   X  m by n matrix whose entries are to be sorted
//   dim  dimensional along which to sort:
//     1  sort each column (matlab default)
//     2  sort each row
//   ascending  sort ascending (true, matlab default) or descending (false)
// Outputs:
//   Y  m by n matrix whose entries are sorted
//   IX  m by n matrix of indices so that if dim = 1, then in matlab notation
//     for j = 1:n, Y(:,j) = X(I(:,j),j); end
template<typename DerivedX, typename DerivedY, typename DerivedIX>
inline void sort(const Eigen::PlainObjectBase<DerivedX> &X,
                 const int dim,
                 const bool ascending,
                 Eigen::PlainObjectBase<DerivedY> &Y,
                 Eigen::PlainObjectBase<DerivedIX> &IX);
// Special case if size(X,dim) == 2
template<typename DerivedX, typename DerivedY, typename DerivedIX>
inline void sort2(const Eigen::PlainObjectBase<DerivedX> &X,
                  const int dim,
                  const bool ascending,
                  Eigen::PlainObjectBase<DerivedY> &Y,
                  Eigen::PlainObjectBase<DerivedIX> &IX);
// Special case if size(X,dim) == 3
template<typename DerivedX, typename DerivedY, typename DerivedIX>
inline void sort3(const Eigen::PlainObjectBase<DerivedX> &X,
                  const int dim,
                  const bool ascending,
                  Eigen::PlainObjectBase<DerivedY> &Y,
                  Eigen::PlainObjectBase<DerivedIX> &IX);

}  // namespace igl

#include "sort.cpp"

#endif
