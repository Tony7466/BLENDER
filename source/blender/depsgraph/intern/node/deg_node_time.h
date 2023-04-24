/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2013 Blender Foundation */

/** \file
 * \ingroup depsgraph
 */

#pragma once

#include "DEG_depsgraph.h"

#include "intern/node/deg_node.h"

namespace blender::deg {

const char *timeSourceTypeAsString(eTimeSourceType source_type);

/* Time Source Node. */
struct TimeSourceNode : public Node {
  TimeSourceNode();

  virtual string identifier() const override;

  // TODO: evaluate() operation needed

  virtual void tag_update(Depsgraph *graph, eUpdateSource source) override;

  void flush_update_tag(Depsgraph *graph);

  /* Type of time source. */
  eTimeSourceType source_type;

  bool tagged_for_update = false;

  /* Frame at which the depsgraph was last evaluated, including subframe. */
  float frame;
  /* Frame at which the depsgraph was last evaluated, after remapping (time stretching). */
  float ctime;

  DEG_DEPSNODE_DECLARE;
};

}  // namespace blender::deg
