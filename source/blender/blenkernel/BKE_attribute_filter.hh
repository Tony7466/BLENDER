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

inline auto attribute_filter_with_extra_skip(AttributeFilter filter, const Span<StringRef> skip)
{
  return [filter, skip](const StringRef name) {
    if (skip.contains(name)) {
      return AttributeFilterResult::AllowSkip;
    }
    return filter(name);
  };
}

template<typename SkipSetT>
inline auto attribute_filter_with_extra_skip_set_ref(AttributeFilter filter, const SkipSetT &skip)
{
  return [filter, &skip](const StringRef name) {
    if (skip.contains_as(name)) {
      return AttributeFilterResult::AllowSkip;
    }
    return filter(name);
  };
}

inline auto attribute_filter_from_skip(const Span<StringRef> skip)
{
  return [skip](const StringRef name) {
    if (skip.contains(name)) {
      return AttributeFilterResult::AllowSkip;
    }
    return AttributeFilterResult::Process;
  };
}

template<typename SkipSetT> inline auto attribute_filter_from_skip_set_ref(const SkipSetT &skip)
{
  return [&skip](const StringRef name) {
    if (skip.contains_as(name)) {
      return AttributeFilterResult::AllowSkip;
    }
    return AttributeFilterResult::Process;
  };
}

/**
 * Checks if the attribute name has the `.a_` prefix which indicates that it is an anonymous
 * attribute. I.e. it is just internally used by Blender and the name should not be exposed to the
 * user.
 *
 * Use #hash_to_anonymous_attribute_name to generate names for anonymous attributes.
 */
inline bool attribute_name_is_anonymous(const StringRef name)
{
  return name.startswith(".a_");
}

static constexpr auto ProcessAllAttributes = [](const StringRef /*name*/) {
  return AttributeFilterResult::Process;
};

static constexpr auto ProcessAllAttributeExceptAnonymous = [](const StringRef name) {
  if (attribute_name_is_anonymous(name)) {
    return AttributeFilterResult::AllowSkip;
  }
  return AttributeFilterResult::Process;
};

}  // namespace blender::bke
