/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

#include "BLI_blenlib.h"

#include "DNA_sequence_types.h"

#include "SEQ_connect.hh"
#include "SEQ_time.hh"

static void seq_connections_free(Sequence *seq)
{
  ListBase *connections = &seq->connections;
  LISTBASE_FOREACH_MUTABLE (SeqConnection *, con, connections) {
    MEM_delete(con);
  }
  BLI_listbase_clear(connections);
}

void SEQ_connections_duplicate(ListBase *connections_dst, ListBase *connections_src)
{
  LISTBASE_FOREACH (SeqConnection *, con, connections_src) {
    SeqConnection *con_duplicate = MEM_cnew<SeqConnection>(__func__, *con);
    BLI_addtail(connections_dst, con_duplicate);
  }
}

bool SEQ_disconnect(Sequence *seq)
{
  if (BLI_listbase_is_empty(&seq->connections)) {
    return false;
  }
  /* Remove `SeqConnections` from other strips' `connections` list that point to `seq`. */
  LISTBASE_FOREACH (SeqConnection *, con_seq, &seq->connections) {
    Sequence *other = con_seq->seq_ref;
    LISTBASE_FOREACH_MUTABLE (SeqConnection *, con_other, &other->connections) {
      if (con_other->seq_ref == seq) {
        BLI_remlink(&other->connections, con_other);
        MEM_delete(con_other);
      }
    }
  }
  /* Now clear `connections` for `seq` itself.*/
  seq_connections_free(seq);

  return true;
}

void SEQ_connect_pair(Sequence *seq1, Sequence *seq2)
{
  blender::VectorSet<Sequence *> seq_list;
  seq_list.add(seq1);
  seq_list.add(seq2);

  SEQ_connect_multiple(seq_list);
}

void SEQ_connect_multiple(blender::VectorSet<Sequence *> &seq_list)
{
  for (Sequence *seq1 : seq_list) {
    SEQ_disconnect(seq1);
    for (Sequence *seq2 : seq_list) {
      if (seq1 == seq2) {
        continue;
      }
      SeqConnection *con = MEM_cnew<SeqConnection>("seqconnection");
      con->seq_ref = seq2;
      BLI_addtail(&seq1->connections, con);
    }
  }
}

bool SEQ_is_strip_connected(const Sequence *seq)
{
  return !BLI_listbase_is_empty(&seq->connections);
}
