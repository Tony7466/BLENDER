/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_geometry_nodes_lazy_function.hh"

#include "BKE_compute_contexts.hh"
#include "BKE_geometry_nodes_closure.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_socket_value.hh"

namespace blender::nodes {

class LazyFunctionForClosureZone : public LazyFunction {
 private:
  const bNodeTree &btree_;
  const bke::bNodeTreeZone &zone_;
  const bNode &repeat_output_bnode_;
  const ZoneBuildInfo &zone_info_;
  const ZoneBodyFunction &body_fn_;

 public:
  LazyFunctionForClosureZone(const bNodeTree &btree,
                             const bke::bNodeTreeZone &zone,
                             ZoneBuildInfo &zone_info,
                             const ZoneBodyFunction &body_fn)
      : btree_(btree),
        zone_(zone),
        repeat_output_bnode_(*zone.output_node),
        zone_info_(zone_info),
        body_fn_(body_fn)
  {
    debug_name_ = "Closure Zone";

    initialize_zone_wrapper(zone, zone_info, body_fn, inputs_, outputs_);

    /* All border links are used. */
    for (const int i : zone_.border_links.index_range()) {
      inputs_[zone_info.indices.inputs.border_links[i]].usage = lf::ValueUsage::Used;
    }
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    params.set_output(zone_info_.indices.outputs.main[0],
                      bke::SocketValueVariant(bke::ClosurePtr()));
    for (const int i : zone_.border_links.index_range()) {
      params.set_output(zone_info_.indices.outputs.border_link_usages[i], true);
    }
  }
};

LazyFunction &build_closure_zone_lazy_function(ResourceScope &scope,
                                               const bNodeTree &btree,
                                               const bke::bNodeTreeZone &zone,
                                               ZoneBuildInfo &zone_info,
                                               const ZoneBodyFunction &body_fn)
{
  return scope.construct<LazyFunctionForClosureZone>(btree, zone, zone_info, body_fn);
}

}  // namespace blender::nodes
