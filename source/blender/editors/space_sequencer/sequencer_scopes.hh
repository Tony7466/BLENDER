/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spseq
 */

#pragma once

#include "BLI_array.hh"
#include "BLI_math_vector_types.hh"

struct ImBuf;

namespace blender::ed::seq {

struct ScopeHistogram {
  Array<uint3> data;
  uint3 max_value;

  void calc_from_ibuf(const ImBuf *ibuf);
};

struct SeqScopes {
  SeqScopes() = default;
  ~SeqScopes();
  SeqScopes(const SeqScopes &) = delete;
  void operator=(const SeqScopes &) = delete;

  void cleanup();

  ImBuf *reference_ibuf = nullptr;
  ImBuf *zebra_ibuf = nullptr;
  ImBuf *waveform_ibuf = nullptr;
  ImBuf *sep_waveform_ibuf = nullptr;
  ImBuf *vector_ibuf = nullptr;

  ScopeHistogram histogram;
};

ImBuf *make_waveform_view_from_ibuf(const ImBuf *ibuf);
ImBuf *make_sep_waveform_view_from_ibuf(const ImBuf *ibuf);
ImBuf *make_vectorscope_view_from_ibuf(const ImBuf *ibuf);
ImBuf *make_zebra_view_from_ibuf(const ImBuf *ibuf, float perc);

}  // namespace blender::ed::seq
