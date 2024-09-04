/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_function_ref.hh"
#include "BLI_string_ref.hh"

namespace blender::bke {

struct AttributeFilter {
 public:
  enum class Result {
    AllowSkip,
    Process,
  };

  virtual ~AttributeFilter() = default;

  virtual Result filter(const StringRef /*name*/) const
  {
    return Result::Process;
  }

  bool allow_skip(const StringRef name) const
  {
    return this->filter(name) == Result::AllowSkip;
  }
};

}  // namespace blender::bke
