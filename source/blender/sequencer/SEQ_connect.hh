/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup sequencer
 */

#include "BLI_vector_set.hh"

struct SeqConnection;
struct Sequence;
struct ListBase;

void SEQ_connections_duplicate(ListBase *connections_dst, ListBase *connections_src);

/**
 * Disconnect the strip from any connections it might have with other strips. This function also
 * frees the allocated memory as necessary. Returns false if the strip was not already connected.
 */
bool SEQ_disconnect(Sequence *seq);

/**
 * Connect two strips `seq1` and `seq2` so that they may be selected together. Any connections the
 * strips already have will be severed before reconnection.
 */
void SEQ_connect_pair(Sequence *seq1, Sequence *seq2);

/**
 * Connect a list of strips so that they may be selected together. Any connections the
 * strips already have will be severed before reconnection.
 */
void SEQ_connect_multiple(blender::VectorSet<Sequence *> seq_list);

/**
 * Check whether a strip has any connections.
 */
bool SEQ_is_strip_connected(Sequence *seq);
