// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "sort.h"

#include <algorithm>
#include <cassert>
#include <iostream>

// For use with functions like std::sort
template<class T> struct IndexLessThan {
  IndexLessThan(const T arr) : arr(arr) {}
  bool operator()(const size_t a, const size_t b) const
  {
    return arr[a] < arr[b];
  }
  const T arr;
};

// This implementation is O(n), but also uses O(n) extra memory
template<class T>
static inline void reorder(const std::vector<T> &unordered,
                           std::vector<size_t> const &index_map,
                           std::vector<T> &ordered)
{
  // copy for the reorder according to index_map, because unsorted may also be
  // sorted
  std::vector<T> copy = unordered;
  ordered.resize(index_map.size());
  for (int i = 0; i < (int)index_map.size(); i++) {
    ordered[i] = copy[index_map[i]];
  }
}

template<class T>
static inline void index_map_sort(const std::vector<T> &unsorted,
                                  const bool ascending,
                                  std::vector<T> &sorted,
                                  std::vector<size_t> &index_map)
{
  // Original unsorted index map
  index_map.resize(unsorted.size());
  for (size_t i = 0; i < unsorted.size(); i++) {
    index_map[i] = i;
  }
  // Sort the index map, using unsorted for comparison
  std::sort(index_map.begin(), index_map.end(), IndexLessThan<const std::vector<T> &>(unsorted));

  // if not ascending then reverse
  if (!ascending) {
    std::reverse(index_map.begin(), index_map.end());
  }
  // make space for output without clobbering
  sorted.resize(unsorted.size());
  // reorder unsorted into sorted using index map
  reorder(unsorted, index_map, sorted);
}

template<typename DerivedX, typename DerivedY, typename DerivedIX>
inline void igl::sort(const Eigen::PlainObjectBase<DerivedX> &X,
                      const int dim,
                      const bool ascending,
                      Eigen::PlainObjectBase<DerivedY> &Y,
                      Eigen::PlainObjectBase<DerivedIX> &IX)
{
  // get number of rows (or columns)
  int num_inner = (dim == 1 ? X.rows() : X.cols());
  // Special case for swapping
  switch (num_inner) {
    default:
      break;
    case 2:
      return igl::sort2(X, dim, ascending, Y, IX);
    case 3:
      return igl::sort3(X, dim, ascending, Y, IX);
  }
  using namespace Eigen;
  // get number of columns (or rows)
  int num_outer = (dim == 1 ? X.cols() : X.rows());
  // dim must be 2 or 1
  assert(dim == 1 || dim == 2);
  // Resize output
  Y.resize(X.rows(), X.cols());
  IX.resize(X.rows(), X.cols());
  // idea is to process each column (or row) as a std vector
  // loop over columns (or rows)
  for (int i = 0; i < num_outer; i++) {
    // Unsorted index map for this column (or row)
    std::vector<size_t> index_map(num_inner);
    std::vector<double> data(num_inner);
    for (int j = 0; j < num_inner; j++) {
      if (dim == 1) {
        data[j] = (double)X(j, i);
      }
      else {
        data[j] = (double)X(i, j);
      }
    }
    // sort this column (or row)
    index_map_sort(data, ascending, data, index_map);
    // Copy into Y and IX
    for (int j = 0; j < num_inner; j++) {
      if (dim == 1) {
        Y(j, i) = data[j];
        IX(j, i) = index_map[j];
      }
      else {
        Y(i, j) = data[j];
        IX(i, j) = index_map[j];
      }
    }
  }
}

template<typename DerivedX, typename DerivedY, typename DerivedIX>
inline void igl::sort2(const Eigen::PlainObjectBase<DerivedX> &X,
                       const int dim,
                       const bool ascending,
                       Eigen::PlainObjectBase<DerivedY> &Y,
                       Eigen::PlainObjectBase<DerivedIX> &IX)
{
  using namespace Eigen;
  using namespace std;
  typedef typename Eigen::PlainObjectBase<DerivedY>::Scalar YScalar;
  Y = X.template cast<YScalar>();
  // get number of columns (or rows)
  int num_outer = (dim == 1 ? X.cols() : X.rows());
  // get number of rows (or columns)
  int num_inner = (dim == 1 ? X.rows() : X.cols());
  assert(num_inner == 2);
  (void)num_inner;
  typedef typename Eigen::PlainObjectBase<DerivedIX>::Scalar Index;
  IX.resize(X.rows(), X.cols());
  if (dim == 1) {
    IX.row(0).setConstant(0);  // = Eigen::PlainObjectBase<DerivedIX>::Zero(1,IX.cols());
    IX.row(1).setConstant(1);  // = Eigen::PlainObjectBase<DerivedIX>::Ones (1,IX.cols());
  }
  else {
    IX.col(0).setConstant(0);  // = Eigen::PlainObjectBase<DerivedIX>::Zero(IX.rows(),1);
    IX.col(1).setConstant(1);  // = Eigen::PlainObjectBase<DerivedIX>::Ones (IX.rows(),1);
  }
  // loop over columns (or rows)
  for (int i = 0; i < num_outer; i++) {
    YScalar &a = (dim == 1 ? Y(0, i) : Y(i, 0));
    YScalar &b = (dim == 1 ? Y(1, i) : Y(i, 1));
    Index &ai = (dim == 1 ? IX(0, i) : IX(i, 0));
    Index &bi = (dim == 1 ? IX(1, i) : IX(i, 1));
    if ((ascending && a > b) || (!ascending && a < b)) {
      std::swap(a, b);
      std::swap(ai, bi);
    }
  }
}

template<typename DerivedX, typename DerivedY, typename DerivedIX>
inline void igl::sort3(const Eigen::PlainObjectBase<DerivedX> &X,
                       const int dim,
                       const bool ascending,
                       Eigen::PlainObjectBase<DerivedY> &Y,
                       Eigen::PlainObjectBase<DerivedIX> &IX)
{
  using namespace Eigen;
  using namespace std;
  typedef typename Eigen::PlainObjectBase<DerivedY>::Scalar YScalar;
  Y = X.template cast<YScalar>();
  // get number of columns (or rows)
  int num_outer = (dim == 1 ? X.cols() : X.rows());
  // get number of rows (or columns)
  int num_inner = (dim == 1 ? X.rows() : X.cols());
  assert(num_inner == 3);
  (void)num_inner;
  typedef typename Eigen::PlainObjectBase<DerivedIX>::Scalar Index;
  IX.resize(X.rows(), X.cols());
  if (dim == 1) {
    IX.row(0).setConstant(0);  // = Eigen::PlainObjectBase<DerivedIX>::Zero(1,IX.cols());
    IX.row(1).setConstant(1);  // = Eigen::PlainObjectBase<DerivedIX>::Ones (1,IX.cols());
    IX.row(2).setConstant(2);  // = Eigen::PlainObjectBase<DerivedIX>::Ones (1,IX.cols());
  }
  else {
    IX.col(0).setConstant(0);  // = Eigen::PlainObjectBase<DerivedIX>::Zero(IX.rows(),1);
    IX.col(1).setConstant(1);  // = Eigen::PlainObjectBase<DerivedIX>::Ones (IX.rows(),1);
    IX.col(2).setConstant(2);  // = Eigen::PlainObjectBase<DerivedIX>::Ones (IX.rows(),1);
  }

  const auto &inner = [&IX, &Y, &dim, &ascending](const Index &i) {
    YScalar &a = (dim == 1 ? Y(0, i) : Y(i, 0));
    YScalar &b = (dim == 1 ? Y(1, i) : Y(i, 1));
    YScalar &c = (dim == 1 ? Y(2, i) : Y(i, 2));
    Index &ai = (dim == 1 ? IX(0, i) : IX(i, 0));
    Index &bi = (dim == 1 ? IX(1, i) : IX(i, 1));
    Index &ci = (dim == 1 ? IX(2, i) : IX(i, 2));
    if (ascending) {
      // 123 132 213 231 312 321
      if (a > b) {
        std::swap(a, b);
        std::swap(ai, bi);
      }
      // 123 132 123 231 132 231
      if (b > c) {
        std::swap(b, c);
        std::swap(bi, ci);
        // 123 123 123 213 123 213
        if (a > b) {
          std::swap(a, b);
          std::swap(ai, bi);
        }
        // 123 123 123 123 123 123
      }
    }
    else {
      // 123 132 213 231 312 321
      if (a < b) {
        std::swap(a, b);
        std::swap(ai, bi);
      }
      // 213 312 213 321 312 321
      if (b < c) {
        std::swap(b, c);
        std::swap(bi, ci);
        // 231 321 231 321 321 321
        if (a < b) {
          std::swap(a, b);
          std::swap(ai, bi);
        }
        // 321 321 321 321 321 321
      }
    }
  };
  parallel_for(num_outer, inner, 16000);
}
