/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup freestyle
 */

#include <iostream>
#include <string>

using namespace std;

namespace Freestyle {

class Iterator {
 public:
  virtual ~Iterator() {}

  virtual string getExactTypeName() const
  {
    return "Iterator";
  }

  virtual int increment()
  {
    cerr << "Warning: increment() not implemented" << endl;
    return 0;
  }

  virtual int decrement()
  {
    cerr << "Warning: decrement() not implemented" << endl;
    return 0;
  }

  virtual bool isBegin() const
  {
    cerr << "Warning: isBegin() not implemented" << endl;
    return false;
  }

  virtual bool isEnd() const
  {
    cerr << "Warning:  isEnd() not implemented" << endl;
    return false;
  }
};

} /* namespace Freestyle */
