/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_anonymous_attribute_id.hh"
#include "BKE_attribute_filter.hh"

#include "BLI_set.hh"

namespace blender::bke {

inline auto attribute_filter_with_skip_ref(AttributeFilter filter, const Span<StringRef> skip)
{
  return [filter, skip](const StringRef name) {
    if (skip.contains(name)) {
      return AttributeFilterResult::AllowSkip;
    }
    return filter(name);
  };
}

template<typename StringT>
inline auto attribute_filter_with_skip_ref(AttributeFilter filter, const Set<StringT> &skip)
{
  return [filter, &skip](const StringRef name) {
    if (skip.contains_as(name)) {
      return AttributeFilterResult::AllowSkip;
    }
    return filter(name);
  };
}

inline auto attribute_filter_from_skip_ref(const Span<StringRef> skip)
{
  return [skip](const StringRef name) {
    if (skip.contains(name)) {
      return AttributeFilterResult::AllowSkip;
    }
    return AttributeFilterResult::Process;
  };
}

template<typename StringT> inline auto attribute_filter_from_skip_ref(const Set<StringT> &skip)
{
  return [&skip](const StringRef name) {
    if (skip.contains_as(name)) {
      return AttributeFilterResult::AllowSkip;
    }
    return AttributeFilterResult::Process;
  };
}

static constexpr auto ProcessAllAttributeExceptAnonymous = [](const StringRef name) {
  if (attribute_name_is_anonymous(name)) {
    return AttributeFilterResult::AllowSkip;
  }
  return AttributeFilterResult::Process;
};

}  // namespace blender::bke
