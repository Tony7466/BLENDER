// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "grad.h"
#include <Eigen/Geometry>
#include <vector>

#include "doublearea.h"

template<typename DerivedV, typename DerivedF>
inline void igl::grad(const Eigen::PlainObjectBase<DerivedV> &V,
                      const Eigen::PlainObjectBase<DerivedF> &F,
                      Eigen::SparseMatrix<typename DerivedV::Scalar> &G,
                      bool uniform)
{
  Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 3> eperp21(F.rows(), 3),
      eperp13(F.rows(), 3);

  for (int i = 0; i < F.rows(); ++i) {
    // renaming indices of vertices of triangles for convenience
    int i1 = F(i, 0);
    int i2 = F(i, 1);
    int i3 = F(i, 2);

    // #F x 3 matrices of triangle edge vectors, named after opposite vertices
    Eigen::Matrix<typename DerivedV::Scalar, 1, 3> v32 = V.row(i3) - V.row(i2);
    Eigen::Matrix<typename DerivedV::Scalar, 1, 3> v13 = V.row(i1) - V.row(i3);
    Eigen::Matrix<typename DerivedV::Scalar, 1, 3> v21 = V.row(i2) - V.row(i1);
    Eigen::Matrix<typename DerivedV::Scalar, 1, 3> n = v32.cross(v13);
    // area of parallelogram is twice area of triangle
    // area of parallelogram is || v1 x v2 ||
    // This does correct l2 norm of rows, so that it contains #F list of twice
    // triangle areas
    double dblA = std::sqrt(n.dot(n));
    Eigen::Matrix<typename DerivedV::Scalar, 1, 3> u;
    if (!uniform) {
      // now normalize normals to get unit normals
      u = n / dblA;
    }
    else {
      // Abstract equilateral triangle v1=(0,0), v2=(h,0), v3=(h/2, (sqrt(3)/2)*h)

      // get h (by the area of the triangle)
      double h = sqrt((dblA) /
                      sin(M_PI / 3.0));  // (h^2*sin(60))/2. = Area => h = sqrt(2*Area/sin_60)

      Eigen::VectorXd v1, v2, v3;
      v1 << 0, 0, 0;
      v2 << h, 0, 0;
      v3 << h / 2., (sqrt(3) / 2.) * h, 0;

      // now fix v32,v13,v21 and the normal
      v32 = v3 - v2;
      v13 = v1 - v3;
      v21 = v2 - v1;
      n = v32.cross(v13);
    }

    // rotate each vector 90 degrees around normal
    double norm21 = std::sqrt(v21.dot(v21));
    double norm13 = std::sqrt(v13.dot(v13));
    eperp21.row(i) = u.cross(v21);
    eperp21.row(i) = eperp21.row(i) / std::sqrt(eperp21.row(i).dot(eperp21.row(i)));
    eperp21.row(i) *= norm21 / dblA;
    eperp13.row(i) = u.cross(v13);
    eperp13.row(i) = eperp13.row(i) / std::sqrt(eperp13.row(i).dot(eperp13.row(i)));
    eperp13.row(i) *= norm13 / dblA;
  }

  std::vector<int> rs;
  rs.reserve(F.rows() * 4 * 3);
  std::vector<int> cs;
  cs.reserve(F.rows() * 4 * 3);
  std::vector<double> vs;
  vs.reserve(F.rows() * 4 * 3);

  // row indices
  for (int r = 0; r < 3; r++) {
    for (int j = 0; j < 4; j++) {
      for (int i = r * F.rows(); i < (r + 1) * F.rows(); i++)
        rs.push_back(i);
    }
  }

  // column indices
  for (int r = 0; r < 3; r++) {
    for (int i = 0; i < F.rows(); i++)
      cs.push_back(F(i, 1));
    for (int i = 0; i < F.rows(); i++)
      cs.push_back(F(i, 0));
    for (int i = 0; i < F.rows(); i++)
      cs.push_back(F(i, 2));
    for (int i = 0; i < F.rows(); i++)
      cs.push_back(F(i, 0));
  }

  // values
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(eperp13(i, 0));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(-eperp13(i, 0));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(eperp21(i, 0));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(-eperp21(i, 0));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(eperp13(i, 1));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(-eperp13(i, 1));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(eperp21(i, 1));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(-eperp21(i, 1));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(eperp13(i, 2));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(-eperp13(i, 2));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(eperp21(i, 2));
  for (int i = 0; i < F.rows(); i++)
    vs.push_back(-eperp21(i, 2));

  // create sparse gradient operator matrix
  G.resize(3 * F.rows(), V.rows());
  std::vector<Eigen::Triplet<typename DerivedV::Scalar>> triplets;
  for (int i = 0; i < (int)vs.size(); ++i) {
    triplets.push_back(Eigen::Triplet<typename DerivedV::Scalar>(rs[i], cs[i], vs[i]));
  }
  G.setFromTriplets(triplets.begin(), triplets.end());
}
