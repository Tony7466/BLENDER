/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup sequencer
 */

#include "BLI_map.hh"
#include "BLI_span.hh"

struct Sequence;
struct SeqRetimingKey;

blender::MutableSpan<SeqRetimingKey> SEQ_retiming_keys_get(const Sequence *seq);
blender::Map<SeqRetimingKey *, Sequence *> SEQ_retiming_selection_get(const struct Editing *ed);
