/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_geo_capture_attribute.hh"
#include "NOD_socket_items_ops.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_enum_types.hh"

#include "BLO_read_write.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_attribute_capture_cc {

NODE_STORAGE_FUNCS(NodeGeometryAttributeCapture)

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Geometry");
  b.add_output<decl::Geometry>("Geometry").propagate_all().align_with_previous();
  if (node != nullptr) {
    const NodeGeometryAttributeCapture &storage = node_storage(*node);
    for (const NodeGeometryAttributeCaptureItem &item :
         Span{storage.capture_items, storage.capture_items_num})
    {
      const eCustomDataType data_type = eCustomDataType(item.data_type);
      const std::string input_identifier =
          CaptureAttributeItemsAccessor::input_socket_identifier_for_item(item);
      const std::string output_identifier =
          CaptureAttributeItemsAccessor::output_socket_identifier_for_item(item);
      b.add_input(data_type, input_identifier).field_on_all();
      b.add_output(data_type, output_identifier).field_on_all().align_with_previous();
    }
  }
  b.add_input<decl::Extend>("", "__extend__");
  b.add_output<decl::Extend>("", "__extend__").align_with_previous();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "domain", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryAttributeCapture *data = MEM_cnew<NodeGeometryAttributeCapture>(__func__);
  data->domain = int8_t(AttrDomain::Point);
  node->storage = data;
}

static void clean_unused_attributes(const AnonymousAttributePropagationInfo &propagation_info,
                                    const Set<AttributeIDRef> &skip,
                                    GeometryComponent &component)
{
  std::optional<MutableAttributeAccessor> attributes = component.attributes_for_write();
  if (!attributes.has_value()) {
    return;
  }

  Vector<std::string> unused_ids;
  attributes->for_all([&](const AttributeIDRef &id, const AttributeMetaData /*meta_data*/) {
    if (!id.is_anonymous()) {
      return true;
    }
    if (skip.contains(id)) {
      return true;
    }
    if (propagation_info.propagate(id.anonymous_id())) {
      return true;
    }
    unused_ids.append(id.name());
    return true;
  });

  for (const std::string &unused_id : unused_ids) {
    attributes->remove(unused_id);
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");

  if (!params.output_is_required("Geometry")) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("The attribute output cannot be used without the geometry output"));
    params.set_default_remaining_outputs();
    return;
  }

  const NodeGeometryAttributeCapture &storage = node_storage(params.node());
  const AttrDomain domain = AttrDomain(storage.domain);

  Vector<const NodeGeometryAttributeCaptureItem *> used_items;
  Vector<GField> fields;
  Vector<AnonymousAttributeIDPtr> attribute_ids;
  Set<AttributeIDRef> used_attribute_ids_set;
  for (const NodeGeometryAttributeCaptureItem &item :
       Span{storage.capture_items, storage.capture_items_num})
  {
    const std::string input_identifier =
        CaptureAttributeItemsAccessor::input_socket_identifier_for_item(item);
    const std::string output_identifier =
        CaptureAttributeItemsAccessor::output_socket_identifier_for_item(item);
    AnonymousAttributeIDPtr attribute_id = params.get_output_anonymous_attribute_id_if_needed(
        output_identifier);
    if (!attribute_id) {
      continue;
    }
    used_attribute_ids_set.add(*attribute_id);
    fields.append(params.extract_input<GField>(input_identifier));
    attribute_ids.append(std::move(attribute_id));
    used_items.append(&item);
  }

  if (fields.is_empty()) {
    params.set_output("Geometry", geometry_set);
    params.set_default_remaining_outputs();
    return;
  }

  const auto capture_on = [&](GeometryComponent &component) {
    for (const int i : fields.index_range()) {
      const AnonymousAttributeID &attribute_id = *attribute_ids[i];
      const GField &field = fields[i];

      /* TODO: Capture all fields at once. */
      bke::try_capture_field_on_geometry(component, attribute_id, domain, field);
    }
    /* Changing of the anonymous attributes may require removing attributes that are no longer
     * needed. */
    clean_unused_attributes(
        params.get_output_propagation_info("Geometry"), used_attribute_ids_set, component);
  };

  /* Run on the instances component separately to only affect the top level of instances. */
  if (domain == AttrDomain::Instance) {
    if (geometry_set.has_instances()) {
      capture_on(geometry_set.get_component_for_write(GeometryComponent::Type::Instance));
    }
  }
  else {
    static const Array<GeometryComponent::Type> types = {GeometryComponent::Type::Mesh,
                                                         GeometryComponent::Type::PointCloud,
                                                         GeometryComponent::Type::Curve,
                                                         GeometryComponent::Type::GreasePencil};

    geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
      for (const GeometryComponent::Type type : types) {
        if (geometry_set.has(type)) {
          capture_on(geometry_set.get_component_for_write(type));
        }
      }
    });
  }

  params.set_output("Geometry", geometry_set);
}

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  return socket_items::try_add_item_via_any_extend_socket<CaptureAttributeItemsAccessor>(
      *ntree, *node, *node, *link);
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<CaptureAttributeItemsAccessor>(*node);
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryAttributeCapture &src_storage = node_storage(*src_node);
  NodeGeometryAttributeCapture *dst_storage = MEM_new<NodeGeometryAttributeCapture>(__func__,
                                                                                    src_storage);
  dst_node->storage = dst_storage;

  socket_items::copy_array<CaptureAttributeItemsAccessor>(*src_node, *dst_node);
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "domain",
                    "Domain",
                    "Which domain to store the data in",
                    rna_enum_attribute_domain_items,
                    NOD_storage_enum_accessors(domain),
                    int8_t(AttrDomain::Point),
                    enums::domain_experimental_grease_pencil_version3_fn,
                    true);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_CAPTURE_ATTRIBUTE, "Capture Attribute", NODE_CLASS_ATTRIBUTE);
  node_type_storage(&ntype, "NodeGeometryAttributeCapture", node_free_storage, node_copy_storage);
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.insert_link = node_insert_link;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_attribute_capture_cc

namespace blender::nodes {

void CaptureAttributeItemsAccessor::blend_write(BlendWriter *writer, const bNode &node)
{
  const auto &storage = *static_cast<const NodeGeometryAttributeCapture *>(node.storage);
  BLO_write_struct_array(
      writer, NodeGeometryAttributeCaptureItem, storage.capture_items_num, storage.capture_items);
  for (const NodeGeometryAttributeCaptureItem &item :
       Span(storage.capture_items, storage.capture_items_num))
  {
    BLO_write_string(writer, item.name);
  }
}

void CaptureAttributeItemsAccessor::blend_read_data(BlendDataReader *reader, bNode &node)
{
  auto &storage = *static_cast<NodeGeometryAttributeCapture *>(node.storage);
  BLO_read_struct_array(
      reader, NodeGeometryAttributeCaptureItem, storage.capture_items_num, &storage.capture_items);
  for (const NodeGeometryAttributeCaptureItem &item :
       Span(storage.capture_items, storage.capture_items_num))
  {
    BLO_read_string(reader, &item.name);
  }
}

}  // namespace blender::nodes
