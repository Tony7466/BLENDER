/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "NOD_socket_search_link.hh"
#include "UI_interface.hh"

#include <fmt/format.h>
#include <regex>
#include <set>

namespace blender::nodes::node_geo_remove_attribute_cc {

NODE_STORAGE_FUNCS(NodeGeometryRemoveNamedAttribute)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_input<decl::String>("Name").is_attribute_name();
  b.add_output<decl::Geometry>("Geometry").propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  const std::string name = params.extract_input<std::string>("Name");

  const NodeGeometryRemoveNamedAttribute &storage = node_storage(params.node());
  const GeometryNodeRemoveNamedAttributeMode mode = (GeometryNodeRemoveNamedAttributeMode)
                                                        storage.mode;
  const bool use_regex = (mode == GEO_NODE_REMOVE_NAMED_ATTRIBUTE_REGEX) ||
                         (mode == GEO_NODE_REMOVE_NAMED_ATTRIBUTE_INV_REGEX);
  const bool inv_regex = (mode == GEO_NODE_REMOVE_NAMED_ATTRIBUTE_INV_REGEX);
  const bool all_trivial = (mode == GEO_NODE_REMOVE_NAMED_ATTRIBUTE_TRIVIAL);

  if (name.empty() && !all_trivial) {
    params.set_output("Geometry", std::move(geometry_set));
    return;
  }
  if (!bke::allow_procedural_attribute_access(name)) {
    params.error_message_add(NodeWarningType::Info, TIP_(bke::no_procedural_access_message));
    params.set_output("Geometry", std::move(geometry_set));
    return;
  }

  std::atomic<bool> attribute_exists = false;
  std::atomic<bool> cannot_delete = false;
  std::mutex m;
  std::set<std::string> deleted_attrs;
  std::set<std::string> kept_attrs;

  std::regex matcher;
  if (use_regex) {
    try {
      matcher = std::regex(name);
    }
    catch (std::regex_error &e) {
      const std::string message = std::string(TIP_("Invalid regex: ")) + TIP_(e.what());
      params.error_message_add(NodeWarningType::Warning, message);
      params.set_output("Geometry", std::move(geometry_set));
      return;
    }
  }

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    for (const GeometryComponent::Type type : {GeometryComponent::Type::Mesh,
                                               GeometryComponent::Type::PointCloud,
                                               GeometryComponent::Type::Curve,
                                               GeometryComponent::Type::Instance})
    {
      if (geometry_set.has(type)) {
        /* First check if the attribute exists before getting write access,
         * to avoid potentially expensive unnecessary copies. */
        const GeometryComponent &read_only_component = *geometry_set.get_component(type);

        if (use_regex) {
          /* Can't keep attrs as AttributeIDRef, because deleting any attribute would
           * invalidate all outstanding AttributeIDRef's from the same component */
          std::set<std::string> attrs;
          for (auto key : read_only_component.attributes()->all_ids())
            if (!key.is_anonymous())
              attrs.insert(key.name());

          bool exists_in_component = false;
          for (auto key : attrs) {
            if (inv_regex ^ std::regex_match(key, matcher)) {
              attribute_exists = true;
              exists_in_component = true;
            }
          }

          if (!exists_in_component) {
            continue;
          }

          GeometryComponent &component = geometry_set.get_component_for_write(type);
          for (auto key : attrs) {
            if (inv_regex ^ std::regex_match(key, matcher)) {
              if (!component.attributes_for_write()->remove(key)) {
                cannot_delete = true;
                m.lock();
                kept_attrs.emplace(key);
                m.unlock();
                continue;
              }
              m.lock();
              deleted_attrs.emplace(key);
              m.unlock();
            }
          }
        }
        else if (all_trivial) {
          std::set<std::string> attrs;
          for (auto key : read_only_component.attributes()->all_ids())
            if (!key.is_anonymous())
              attrs.insert(key.name());

          GeometryComponent &component = geometry_set.get_component_for_write(type);
          for (auto key : attrs) {
            auto metadata = component.attributes()->lookup_meta_data(key);
            if (!metadata) {
              continue;
            }
            eCustomDataType type = metadata->data_type;
            GVArray &array = *(component.attributes()->lookup(key));
            if (array.is_empty()) {
              continue;
            }
            if (type == CD_PROP_FLOAT) {
              auto arr = array.typed<float>();
              bool trivial = true;
              for (auto i = 0; i < arr.size(); i++) {
                if (arr[i] != 0.0f) {
                  trivial = false;
                  break;
                }
              }
              if (!trivial)
                continue;
            }
            else if (type == CD_PROP_FLOAT3) {
              auto arr = array.typed<float3>();
              bool trivial = true;
              for (auto i = 0; i < arr.size(); i++) {
                if (arr[i] != float3(0.0f, 0.0f, 0.0f)) {
                  trivial = false;
                  break;
                }
              }
              if (!trivial)
                continue;
            }
            else {
              continue;
            }
            if (!component.attributes_for_write()->remove(key)) {
              cannot_delete = true;
              m.lock();
              kept_attrs.emplace(key);
              m.unlock();
            }
            else {
              m.lock();
              deleted_attrs.emplace(key);
              m.unlock();
            }
          }
        }
        else {
          if (read_only_component.attributes()->contains(name)) {
            attribute_exists = true;
          }
          else {
            continue;
          }

          GeometryComponent &component = geometry_set.get_component_for_write(type);
          if (!component.attributes_for_write()->remove(name)) {
            cannot_delete = true;
          }
        }
      }
    }
  });

  if (use_regex || all_trivial) {
    for (auto attr : deleted_attrs) {
      if (kept_attrs.find(attr) == kept_attrs.end())
        params.used_named_attribute(attr, NamedAttributeUsage::Remove);
    }
  }
  else {
    if (attribute_exists && !cannot_delete) {
      params.used_named_attribute(name, NamedAttributeUsage::Remove);
    }

    if (!attribute_exists) {
      const std::string message = fmt::format(TIP_("Attribute does not exist: \"{}\""), name);
      params.error_message_add(NodeWarningType::Warning, message);
    }
    if (cannot_delete) {
      const std::string message = fmt::format(TIP_("Cannot delete built-in attribute: \"{}\""),
                                              name);
      params.error_message_add(NodeWarningType::Warning, message);
    }
  }

  params.set_output("Geometry", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  static EnumPropertyItem mode_items[] = {
      {GEO_NODE_REMOVE_NAMED_ATTRIBUTE_EXACT,
       "EXACT",
       0,
       "Exact",
       "Remove the attribute with specified name"},
      {GEO_NODE_REMOVE_NAMED_ATTRIBUTE_REGEX,
       "REGEX",
       0,
       "Regex",
       "Remove all attributes matching the regular expression"},
      {GEO_NODE_REMOVE_NAMED_ATTRIBUTE_INV_REGEX,
       "INV_REGEX",
       0,
       "Inverse regex",
       "Remove all attributes not matching the regular expression"},
      {GEO_NODE_REMOVE_NAMED_ATTRIBUTE_TRIVIAL,
       "TRIVIAL",
       0,
       "Trivial",
       "Remove all named float and vector attributes with null content ('name' parameter "
       "ignored)"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "Which attributes to remove",
                    mode_items,
                    NOD_storage_enum_accessors(mode),
                    GEO_NODE_REMOVE_NAMED_ATTRIBUTE_EXACT);
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryRemoveNamedAttribute *data = MEM_cnew<NodeGeometryRemoveNamedAttribute>(__func__);

  data->mode = GEO_NODE_REMOVE_NAMED_ATTRIBUTE_EXACT;
  node->storage = data;
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_REMOVE_ATTRIBUTE, "Remove Named Attribute", NODE_CLASS_ATTRIBUTE);
  ntype.declare = node_declare;
  bke::node_type_size(&ntype, 170, 100, 700);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  node_type_storage(&ntype,
                    "NodeGeometryRemoveNamedAttribute",
                    node_free_standard_storage,
                    node_copy_standard_storage);
  ntype.initfunc = node_init;

  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_remove_attribute_cc
