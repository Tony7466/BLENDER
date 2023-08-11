/* SPDX-FileCopyrightText: 2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup sequencer
 */

#include "BLI_vector.hh"

struct bContext;
struct Sequence;
struct SeqRetimingHandle;

blender::Vector<Sequence *> sequencer_visible_strips_get(const struct bContext *C);
bool retiming_last_key_is_clicked(struct bContext *C,
                                  const struct Sequence *seq,
                                  const int mval[2]);

const struct SeqRetimingHandle *retiming_mousover_key_get(struct bContext *C,
                                                          const int mval[2],
                                                          Sequence **r_seq);
bool sequencer_retiming_tool_is_active(const struct bContext *C);
void sequencer_draw_retiming(const struct bContext *C);
