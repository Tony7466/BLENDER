#include "NOD_rna_define.hh"

#include "BKE_layer.hh"
#include "BLI_math_matrix.hh"
#include "GEO_transform.hh"
#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BKE_instances.hh"

#include "BKE_geometry_set_instances.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_enum_types.hh"

#include "BKE_camera.h"
#include "BKE_light.h"
#include "BKE_mesh.h"
#include "BKE_object.hh"

#include "BLI_vector.hh"
#include "BLI_virtual_array.hh"
#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"
#include "DNA_camera_types.h"
#include "DNA_collection_types.h"
#include "DNA_light_types.h"
#include "DNA_object_types.h"
#include "node_geometry_util.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_path.hh"
#include "RNA_types.hh"
#include "intern/rna_internal.hh"

#include <any>
#include <memory>
#include <typeindex>
namespace blender::nodes::node_geo_object_attribute_cc {

NODE_STORAGE_FUNCS(GeometryNodeObjectAttribute)

PointerRNA *get_object_type_rna_pointer(Object *object)
{
  PointerRNA object_ptr;
  PointerRNA object_type_ptr;

  Light *light = static_cast<Light *>(object->data);
  Camera *camera = static_cast<Camera *>(object->data);
  Mesh *mesh = static_cast<Mesh *>(object->data);

  if (light != nullptr) {
    object_type_ptr = RNA_id_pointer_create(&light->id);
  }
  else if (camera != nullptr) {
    object_type_ptr = RNA_id_pointer_create(&camera->id);
  }
  else if (mesh != nullptr) {
    object_type_ptr = RNA_id_pointer_create(&mesh->id);
  }

  return &object_type_ptr;
}

static std::any get_attribute_value(PointerRNA *ptr, PropertyRNA *prop)
{

  switch (prop->type) {
    case PROP_FLOAT: {
      if (prop->totarraylength == 4) {
        float value[4];
        RNA_property_float_get_array(ptr, prop, value);
        return (math::Quaternion)value;
      }
      else if (prop->totarraylength == 3) {
        float3 value;
        RNA_property_float_get_array(ptr, prop, value);
        return float3(value);
      }
      else
        return (float)RNA_property_float_get(ptr, prop);
    }
    case PROP_INT: {
      return (int)RNA_property_int_get(ptr, prop);
    }
    case PROP_BOOLEAN: {
      if (prop->totarraylength == 3) {
        bool value[3];
        RNA_property_boolean_get_array(ptr, prop, value);
        return float3(value);
      }
      return (bool)RNA_property_boolean_get(ptr, prop);
    }
    default: {
      return 0.0f;
    }
  }
}

static void node_declare(NodeDeclarationBuilder &b)
{

  const bNode *node = b.node_or_null();

  b.add_input<decl::Object>("Object");
  b.add_input<decl::String>("Name");

  if (node != nullptr) {
    const GeometryNodeObjectAttribute &storage = node_storage(*node);
    const eCustomDataType data_type = eCustomDataType(storage.data_type);
    b.add_output(data_type, "Value");
  }
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_geo_init_attributes(bNodeTree * /*ntree*/, bNode *node)
{
  GeometryNodeObjectAttribute *attr = MEM_cnew<GeometryNodeObjectAttribute>(
      "GeometryNodeAttribute");

  attr->data_type = CD_PROP_FLOAT;

  node->storage = attr;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometryNodeObjectAttribute *attr = static_cast<GeometryNodeObjectAttribute *>(
      params.node().storage);

  Object *object = params.extract_input<Object *>("Object");
  const Object *self_object = params.self_object();
  if (object == nullptr) {
    params.set_default_remaining_outputs();
    return;
  }
  std::string name = params.extract_input<std::string>("Name");
  if (name.empty()) {
    params.set_default_remaining_outputs();
    return;
  }
  PointerRNA object_ptr = RNA_id_pointer_create(&object->id);
  std::any value;
  PointerRNA ptr;
  PropertyRNA *property;
  if (RNA_path_resolve_property(
          get_object_type_rna_pointer(object), name.c_str(), &ptr, &property) ||
      RNA_path_resolve_property(&object_ptr, name.c_str(), &ptr, &property))
  {
    value = get_attribute_value(&ptr, property);
  }

  if (value.has_value()) {
    const std::type_index value_type = value.type();
    if (value_type == typeid(float3)) {
      attr->data_type = CD_PROP_FLOAT3;
      params.set_output("Value", std::any_cast<float3>(value));
    }
    else if (value_type == typeid(math::Quaternion)) {
      params.set_output("Value", std::any_cast<math::Quaternion>(value));
    }
    else if (value_type == typeid(int)) {
      params.set_output("Value", std::any_cast<int>(value));
    }
    else if (value_type == typeid(float4x4)) {
      params.set_output("Value", std::any_cast<float4x4>(value));
    }
    else {
      params.set_output<float>("Value", std::any_cast<float>(value));
    }
  }
  params.set_default_remaining_outputs();
  return;
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "The data type used to read the values",
                    rna_enum_attribute_type_items,
                    NOD_storage_enum_accessors(data_type),
                    CD_PROP_FLOAT,
                    enums::attribute_type_type_with_socket_fn);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_OBJECT_ATTRIBUTE, "Object Attribute", NODE_CLASS_INPUT);

  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_geo_init_attributes;
  node_type_storage(&ntype,
                    "GeometryNodeObjectAttribute",
                    node_free_standard_storage,
                    node_copy_standard_storage);
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_type_size_preset(&ntype, blender::bke::eNodeSizePreset::MIDDLE);
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_object_attribute_cc
