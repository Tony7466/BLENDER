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

// For use with functions like std::sort
template<class T> struct IndexLessThan {
  IndexLessThan(const T arr) : arr(arr) {}
  bool operator()(const size_t a, const size_t b) const
  {
    return arr[a] < arr[b];
  }
  const T arr;
};

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

// Act like matlab's [Y,I] = SORT(X) for std library vectors
// Templates:
//   T  should be a class that implements the '<' comparator operator
// Input:
//   unsorted  unsorted vector
//   ascending  sort ascending (true, matlab default) or descending (false)
// Output:
//   sorted     sorted vector, allowed to be same as unsorted
//   index_map  an index map such that sorted[i] = unsorted[index_map[i]]
template<class T>
inline void sort(const std::vector<T> &unsorted,
                 const bool ascending,
                 std::vector<T> &sorted,
                 std::vector<size_t> &index_map);

// Act like matlab's Y = X(I) for std vectors
// where I contains a vector of indices so that after,
// Y[j] = X[I[j]] for index j
// this implies that Y.size() == I.size()
// X and Y are allowed to be the same reference
template<class T>
inline void reorder(const std::vector<T> &unordered,
                    std::vector<size_t> const &index_map,
                    std::vector<T> &ordered);

}  // namespace igl

#include "sort.cpp"

#endif
