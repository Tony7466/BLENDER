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
  SeqRetimingHandle *key;
  RetimingSelectionElem(Sequence *seq, SeqRetimingHandle *key) : seq(seq), key(key) {}
} RetimingSelectionElem;

blender::MutableSpan<SeqRetimingHandle> SEQ_retiming_keys_get(const Sequence *seq);
blender::Vector<RetimingSelectionElem> SEQ_retiming_selection_get(const struct Scene *scene);
