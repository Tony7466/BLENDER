/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spseq
 */

#pragma once

struct ImBuf;

ImBuf *make_waveform_view_from_ibuf(ImBuf *ibuf);
ImBuf *make_sep_waveform_view_from_ibuf(ImBuf *ibuf);
ImBuf *make_vectorscope_view_from_ibuf(ImBuf *ibuf);
ImBuf *make_zebra_view_from_ibuf(ImBuf *ibuf, float perc);
ImBuf *make_histogram_view_from_ibuf(ImBuf *ibuf);
