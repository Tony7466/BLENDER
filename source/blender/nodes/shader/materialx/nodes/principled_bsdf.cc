/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "principled_bsdf.h"

#include <BKE_node_runtime.hh>

namespace blender::nodes::materialx {

const MaterialX::Color3 MaterialXPrincipledBSDFNode::default_white_color_{1.0, 1.0, 1.0};

MaterialXPrincipledBSDFNode::MaterialXPrincipledBSDFNode(MaterialX::DocumentPtr doc,
                                                         const Depsgraph *depsgraph,
                                                         const Material *material,
                                                         const bNode *node)
    : MaterialXNode(doc, depsgraph, material, node)
{
  matx_node = doc->addNode(
      "standard_surface", MaterialX::createValidName(node->name), "surfaceshader");
}

MaterialX::NodePtr MaterialXPrincipledBSDFNode::convert()
{
#pragma region get inputs
  const float *base_color =
      node->input_by_identifier("Base Color").default_value_typed<bNodeSocketValueRGBA>()->value;
  const float subsurface =
      node->input_by_identifier("Subsurface").default_value_typed<bNodeSocketValueFloat>()->value;

  const float *subsurface_radius = node->input_by_identifier("Subsurface Radius")
                                       .default_value_typed<bNodeSocketValueVector>()
                                       ->value;
  const float *subsurface_color = node->input_by_identifier("Subsurface Color")
                                      .default_value_typed<bNodeSocketValueRGBA>()
                                      ->value;
  const float metallic =
      node->input_by_identifier("Metallic").default_value_typed<bNodeSocketValueFloat>()->value;
  const float specular =
      node->input_by_identifier("Specular").default_value_typed<bNodeSocketValueFloat>()->value;
  const float roughness =
      node->input_by_identifier("Roughness").default_value_typed<bNodeSocketValueFloat>()->value;
  const float anisotropic =
      node->input_by_identifier("Anisotropic").default_value_typed<bNodeSocketValueFloat>()->value;
  const float anisotropic_rot = node->input_by_identifier("Anisotropic Rotation")
                                    .default_value_typed<bNodeSocketValueFloat>()
                                    ->value;
  const float sheen =
      node->input_by_identifier("Sheen").default_value_typed<bNodeSocketValueFloat>()->value;
  const float clearcoat =
      node->input_by_identifier("Clearcoat").default_value_typed<bNodeSocketValueFloat>()->value;
  const float clearcoat_roughness = node->input_by_identifier("Clearcoat Roughness")
                                        .default_value_typed<bNodeSocketValueFloat>()
                                        ->value;
  const float IOR =
      node->input_by_identifier("IOR").default_value_typed<bNodeSocketValueFloat>()->value;
  const float transmission = node->input_by_identifier("Transmission")
                                 .default_value_typed<bNodeSocketValueFloat>()
                                 ->value;
  const float *emission =
      node->input_by_identifier("Emission").default_value_typed<bNodeSocketValueRGBA>()->value;
  const float emission_str = node->input_by_identifier("Emission Strength")
                                 .default_value_typed<bNodeSocketValueFloat>()
                                 ->value;
  const float *normal =
      node->input_by_identifier("Normal").default_value_typed<bNodeSocketValueVector>()->value;
  const float *clearcoat_normal = node->input_by_identifier("Clearcoat Normal")
                                      .default_value_typed<bNodeSocketValueVector>()
                                      ->value;
  const float *tangent =
      node->input_by_identifier("Tangent").default_value_typed<bNodeSocketValueVector>()->value;
#pragma endregion get inputs

#pragma region set inputs
  matx_node->addInput("base", "float")->setValue(1.0);
  matx_node->addInput("base_color", "color3")
      ->setValue(MaterialX::Color3(base_color[0], base_color[1], base_color[2]));
  matx_node->addInput("diffuse_roughness", "float")->setValue(roughness);
  matx_node->addInput("normal", "vector3")
      ->setValue(MaterialX::Vector3(normal[0], normal[1], normal[2]));
  matx_node->addInput("tangent", "vector3")
      ->setValue(MaterialX::Vector3(tangent[0], tangent[1], tangent[2]));

  matx_node->addInput("metalness", "float")->setValue(metallic);

  matx_node->addInput("specular", "float")->setValue(specular);
  matx_node->addInput("specular_color", "color3")->setValue(default_white_color_);
  matx_node->addInput("specular_roughness", "float")->setValue(roughness);
  matx_node->addInput("specular_IOR", "float")->setValue(IOR);
  matx_node->addInput("specular_anisotropy", "float")->setValue(anisotropic);
  matx_node->addInput("specular_rotation", "float")->setValue(anisotropic_rot);

  matx_node->addInput("transmission", "float")->setValue(transmission);
  matx_node->addInput("transmission_color", "color3")->setValue(default_white_color_);
  matx_node->addInput("transmission_extra_roughness", "float")->setValue(roughness);

  matx_node->addInput("subsurface", "float")->setValue(subsurface);
  matx_node->addInput("subsurface_color", "color3")
      ->setValue(MaterialX::Color3(subsurface_color[0], subsurface_color[1], subsurface_color[2]));
  matx_node->addInput("subsurface_radius", "color3")
      ->setValue(
          MaterialX::Color3(subsurface_radius[0], subsurface_radius[1], subsurface_radius[2]));
  matx_node->addInput("subsurface_anisotropy", "float")->setValue(anisotropic);

  matx_node->addInput("sheen", "float")->setValue(sheen);
  matx_node->addInput("sheen_color", "color3")->setValue(default_white_color_);
  matx_node->addInput("sheen_roughness", "float")->setValue(roughness);

  matx_node->addInput("coat", "float")->setValue(clearcoat);
  matx_node->addInput("coat_color", "color3")->setValue(default_white_color_);
  matx_node->addInput("coat_roughness", "float")->setValue(clearcoat_roughness);
  matx_node->addInput("coat_IOR", "float")->setValue(IOR);
  matx_node->addInput("coat_anisotropy", "float")->setValue(anisotropic);
  matx_node->addInput("coat_rotation", "float")->setValue(anisotropic_rot);
  matx_node->addInput("coat_normal", "vector3")
      ->setValue(
          MaterialX::Vector3(clearcoat_normal[0], clearcoat_normal[1], clearcoat_normal[2]));

  matx_node->addInput("emission", "float")->setValue(emission_str);
  matx_node->addInput("emission_color", "color3")
      ->setValue(MaterialX::Color3(emission[0], emission[1], emission[2]));
#pragma endregion set inputs
  return matx_node;
}

}  // namespace blender::nodes::materialx
