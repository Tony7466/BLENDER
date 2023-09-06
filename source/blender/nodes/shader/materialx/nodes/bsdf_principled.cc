/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem BSDFPrincipledNodeParser::compute()
{
  NodeItem base_color = get_input_value("Base Color", NodeItem::Type::Color3);

  NodeItem subsurface = get_input_value("Subsurface", NodeItem::Type::Float);
  NodeItem subsurface_radius = get_input_value("Subsurface Radius", NodeItem::Type::Color3);
  NodeItem subsurface_color = get_input_value("Subsurface Color", NodeItem::Type::Color3);

  NodeItem metallic = get_input_value("Metallic", NodeItem::Type::Float);
  NodeItem specular = get_input_value("Specular", NodeItem::Type::Float);
  // NodeItem specular_tint = get_input_value("Specular Tint");
  NodeItem roughness = get_input_value("Roughness", NodeItem::Type::Float);

  /* TODO: use Specular Tint input */
  NodeItem anisotropic = get_input_value("Anisotropic", NodeItem::Type::Float);
  NodeItem anisotropic_rotation = get_input_value("Anisotropic Rotation", NodeItem::Type::Float);
  // anisotropic_rotation = 0.5 - (anisotropic_rotation % 1.0)

  NodeItem sheen = get_input_value("Sheen", NodeItem::Type::Float);
  // sheen_tint = get_input_value("Sheen Tint");

  NodeItem clearcoat = get_input_value("Clearcoat", NodeItem::Type::Float);
  NodeItem clearcoat_roughness = get_input_value("Clearcoat Roughness", NodeItem::Type::Float);

  NodeItem ior = get_input_value("IOR", NodeItem::Type::Float);

  NodeItem transmission = get_input_value("Transmission", NodeItem::Type::Float);

  NodeItem emission = get_input_value("Emission", NodeItem::Type::Color3);
  NodeItem emission_strength = get_input_value("Emission Strength", NodeItem::Type::Float);

  NodeItem alpha = get_input_value("Alpha", NodeItem::Type::Float);
  // transparency = 1.0 - alpha

  NodeItem normal = get_input_link("Normal");
  NodeItem clearcoat_normal = get_input_link("Clearcoat Normal");
  NodeItem tangent = get_input_link("Tangent");

  /* Creating standard_surface */
  NodeItem res = create_node("standard_surface", "surfaceshader");
  res.set_input("base", 1.0, "float");
  res.set_input("base_color", base_color);
  res.set_input("diffuse_roughness", roughness);
  if (normal) {
    res.set_input("normal", normal);
  }
  if (tangent) {
    res.set_input("tangent", tangent);
  }
  res.set_input("metalness", metallic);

  res.set_input("specular", specular);
  res.set_input("specular_color", base_color);
  res.set_input("specular_roughness", roughness);
  res.set_input("specular_IOR", ior);
  res.set_input("specular_anisotropy", anisotropic);
  res.set_input("specular_rotation", anisotropic_rotation);

  res.set_input("transmission", transmission);
  res.set_input("transmission_color", base_color);
  res.set_input("transmission_extra_roughness", roughness);

  res.set_input("subsurface", subsurface);
  res.set_input("subsurface_color", subsurface_color);
  res.set_input("subsurface_radius", subsurface_radius);
  res.set_input("subsurface_anisotropy", anisotropic);

  res.set_input("sheen", sheen);
  res.set_input("sheen_color", base_color);
  res.set_input("sheen_roughness", roughness);

  res.set_input("coat", clearcoat);
  res.set_input("coat_color", base_color);
  res.set_input("coat_roughness", clearcoat_roughness);
  res.set_input("coat_IOR", ior);
  res.set_input("coat_anisotropy", anisotropic);
  res.set_input("coat_rotation", anisotropic_rotation);
  if (clearcoat_normal) {
    res.set_input("coat_normal", clearcoat_normal);
  }

  res.set_input("emission", emission_strength);
  res.set_input("emission_color", emission);

  return res;
}

}  // namespace blender::nodes::materialx
