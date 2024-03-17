/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_generic_virtual_array.hh"

namespace blender::implicit_sharing::unshare {

class ArrayUnsharePolicy {
 public:
  void unshare_array(const GVArray &src, GMutableSpan dst) const
  {
    BLI_assert(src.type() == dst.type());
    BLI_assert(src.size() == dst.size());
    this->unshare_array_impl(src, dst);
  }

 private:
  virtual void unshare_array_impl(const GVArray &src, GMutableSpan dst) const = 0;
};

class CopyAll : public ArrayUnsharePolicy {
  virtual void unshare_array_impl(const GVArray &src, GMutableSpan dst) const override
  {
    src.materialize_to_uninitialized(dst.data());
  }
};

class IgnoreOldValues : public ArrayUnsharePolicy {
  void unshare_array_impl(const GVArray & /*src*/, GMutableSpan dst) const override
  {
    const CPPType &type = dst.type();
    type.default_construct_n(dst.data(), dst.size());
  }
};

using DefaultArrayUnsharePolicy = CopyAll;

}  // namespace blender::implicit_sharing::unshare

namespace blender {
using implicit_sharing::unshare::ArrayUnsharePolicy;
using implicit_sharing::unshare::DefaultArrayUnsharePolicy;
}  // namespace blender
