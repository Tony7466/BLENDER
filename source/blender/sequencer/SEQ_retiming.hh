/* SPDX-FileCopyrightText: 2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup sequencer
 */

#include "BLI_span.hh"

struct Sequence;
struct SeqRetimingHandle;

typedef struct RetimingSelectionElem {
  Sequence *seq;
  SeqRetimingHandle *handle;
  RetimingSelectionElem(Sequence *seq, SeqRetimingHandle *handle) : seq(seq), handle(handle) {}
} RetimingSelectionElem;

blender::MutableSpan<SeqRetimingHandle> SEQ_retiming_handles_get(const Sequence *seq);
blender::Vector<RetimingSelectionElem> SEQ_retiming_selection_get(const struct Scene *scene);
