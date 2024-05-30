/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd_utils.hh"

#include <pxr/base/tf/stringUtils.h>
#include <pxr/base/tf/unicodeUtils.h>

namespace blender::io::usd {

std::string make_safe_name(const std::string &name, [[maybe_unused]] bool allow_unicode)
{
#if PXR_VERSION >= 2403
  if (!allow_unicode) {
    return pxr::TfMakeValidIdentifier(name);
  }

  if (name.empty()) {
    return "_";
  }

  const pxr::TfUtf8CodePoint cp_underscore = pxr::TfUtf8CodePointFromAscii('_');

  bool first = true;
  std::stringstream str;
  for (auto cp : pxr::TfUtf8CodePointView{name}) {
    const bool cp_allowed = first ? (cp == cp_underscore || pxr::TfIsUtf8CodePointXidStart(cp)) :
                                    pxr::TfIsUtf8CodePointXidContinue(cp);
    if (!cp_allowed) {
      str << '_';
    }
    else {
      str << cp;
    }

    first = false;
  }

  return str.str();
#else
  return pxr::TfMakeValidIdentifier(name);
#endif
}

}  // namespace blender::io::usd
