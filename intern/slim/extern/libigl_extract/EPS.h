// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_EPS_H
#define IGL_EPS_H

namespace igl {
// Define a standard value for double epsilon
const double DOUBLE_EPS = 1.0e-14;
const double DOUBLE_EPS_SQ = 1.0e-28;
const float FLOAT_EPS = 1.0e-7;
const float FLOAT_EPS_SQ = 1.0e-14;
// Function returning EPS for corresponding type
template<typename S_type> inline S_type EPS();
template<typename S_type> inline S_type EPS_SQ();
// Template specializations for float and double
template<> inline float EPS<float>();
template<> inline double EPS<double>();
template<> inline float EPS_SQ<float>();
template<> inline double EPS_SQ<double>();
}  // namespace igl

#include "EPS.cpp"

#endif
