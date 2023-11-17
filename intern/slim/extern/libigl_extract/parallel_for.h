// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2016 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_PARALLEL_FOR_H
#define IGL_PARALLEL_FOR_H
#include "igl_inline.h"
#include <functional>

namespace igl {
// PARALLEL_FOR Functional implementation of a basic, open-mp style, parallel
// for loop. If the inner block of a for-loop can be rewritten/encapsulated in
// a single (anonymous/lambda) function call `func` so that the serial code
// looks like:
//
//     for(int i = 0;i<loop_size;i++)
//     {
//       func(i);
//     }
//
// then `parallel_for(loop_size,func,min_parallel)` will use as many threads as
// available on the current hardware to parallelize this for loop so long as
// loop_size<min_parallel, otherwise it will just use a serial for loop.
//
// Inputs:
//   loop_size  number of iterations. I.e. for(int i = 0;i<loop_size;i++) ...
//   func  function handle taking iteration index as only arguement to compute
//     inner block of for loop I.e. for(int i ...){ func(i); }
//   min_parallel  min size of loop_size such that parallel (non-serial)
//     thread pooling should be attempted {0}
// Returns true iff thread pool was invoked
template<typename Index, typename FunctionType>
inline bool parallel_for(const Index loop_size,
                         const FunctionType &func,
                         const size_t min_parallel = 0);
}  // namespace igl

// Implementation

#ifdef _WIN32
#  define _USE_MATH_DEFINES
#endif

#include <algorithm>
#include <cassert>
#include <cmath>
#include <thread>
#include <vector>

#include <BLI_task.hh>

template<typename Index, typename FunctionType>
inline bool igl::parallel_for(const Index loop_size,
                              const FunctionType &func,
                              const size_t min_parallel)
{
  using namespace blender;

  threading::parallel_for(IndexRange(loop_size), min_parallel, [&func](const IndexRange range) {
    for (const Index i : range) {
      func(i);
    }
  });

  return true;
}

#endif
