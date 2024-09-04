/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_function_ref.hh"
#include "BLI_string_ref.hh"

namespace blender::bke {

enum class AttributeFilterResult {
  AllowSkip,
  Process,
};

using AttributeFilter = FunctionRef<AttributeFilterResult(StringRef name)>;

inline bool allow_skipping_attribute(const AttributeFilter &filter, const StringRef name)
{
  return filter(name) == AttributeFilterResult::AllowSkip;
}

static constexpr auto ProcessAllAttributes = [](const StringRef /*name*/) {
  return AttributeFilterResult::Process;
};

}  // namespace blender::bke
