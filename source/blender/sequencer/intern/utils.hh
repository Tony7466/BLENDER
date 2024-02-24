/* SPDX-FileCopyrightText: 2004 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup sequencer
 */

struct ListBase;
struct Scene;

bool sequencer_seq_generates_image(Sequence *seq);
Sequence *SEQ_get_meta_by_seqbase(ListBase *seqbase_main, ListBase *meta_seqbase);
