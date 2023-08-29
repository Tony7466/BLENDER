/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem BSDFPrincipledNodeParser::compute()
{
  auto enabled = [](NodeItem &val) -> bool {
    if (val.node) {
      return true;
    }
    if (!val.value) {
      return false;
    }
    if (val.value->isA<float>()) {
      return val.value->asA<float>() != 0.0f;
    }
    if (val.value->isA<MaterialX::Color4>()) {
      auto c = val.value->asA<MaterialX::Color4>();
      return c[0] != 0.0f || c[1] != 0.0f || c[2] != 0.0f;
    }
    return true;
  };

  /* Getting required inputs
   * Note: if some inputs are not needed they won't be taken */
  NodeItem base_color = get_input_value("Base Color");

  NodeItem subsurface = get_input_value("Subsurface");
  NodeItem subsurface_radius = empty_value();
  NodeItem subsurface_color = empty_value();
  if (enabled(subsurface)) {
    subsurface_radius = get_input_value("Subsurface Radius");
    subsurface_color = get_input_value("Subsurface Color");
  }

  NodeItem metallic = get_input_value("Metallic");
  NodeItem specular = get_input_value("Specular");
  // NodeItem specular_tint = get_input_value("Specular Tint");
  NodeItem roughness = get_input_value("Roughness");

  NodeItem anisotropic = empty_value();
  NodeItem anisotropic_rotation = empty_value();
  if (enabled(metallic)) {
    /* TODO: use Specular Tint input */
    anisotropic = get_input_value("Anisotropic");
    if (enabled(anisotropic)) {
      anisotropic_rotation = get_input_value("Anisotropic Rotation");
      // anisotropic_rotation = 0.5 - (anisotropic_rotation % 1.0)
    }
  }

  NodeItem sheen = get_input_value("Sheen");
  // sheen_tint = empty_value();
  // if enabled(sheen):
  //     sheen_tint = get_input_value("Sheen Tint");

  NodeItem clearcoat = get_input_value("Clearcoat");
  NodeItem clearcoat_roughness = empty_value();
  if (enabled(clearcoat)) {
    clearcoat_roughness = get_input_value("Clearcoat Roughness");
  }

  NodeItem ior = get_input_value("IOR");

  NodeItem transmission = get_input_value("Transmission");
  NodeItem transmission_roughness = empty_value();
  if (enabled(transmission)) {
    transmission_roughness = get_input_value("Transmission Roughness");
  }

  NodeItem emission = get_input_value("Emission");
  NodeItem emission_strength = get_input_value("Emission Strength");

  NodeItem alpha = get_input_value("Alpha");
  // transparency = 1.0 - alpha

  NodeItem normal = get_input_link("Normal");
  NodeItem clearcoat_normal = get_input_link("Clearcoat Normal");
  NodeItem tangent = get_input_link("Tangent");

  /* Creating standard_surface */
  NodeItem res = create_node("standard_surface", "surfaceshader");
  res.set_input("base", 1.0, "float");
  res.set_input("base_color", base_color.to_color3());
  res.set_input("diffuse_roughness", roughness);
  res.set_input("normal", normal);
  res.set_input("tangent", tangent);

  if (enabled(metallic)) {
    res.set_input("metalness", metallic);
  }

  if (enabled(specular)) {
    res.set_input("specular", specular);
    res.set_input("specular_color", base_color.to_color3());
    res.set_input("specular_roughness", roughness);
    res.set_input("specular_IOR", ior);
    res.set_input("specular_anisotropy", anisotropic);
    res.set_input("specular_rotation", anisotropic_rotation);
  }

  if (enabled(transmission)) {
    res.set_input("transmission", transmission);
    res.set_input("transmission_color", base_color.to_color3());
    res.set_input("transmission_extra_roughness", transmission_roughness);
  }

  if (enabled(subsurface)) {
    res.set_input("subsurface", subsurface);
    res.set_input("subsurface_color", subsurface_color);
    res.set_input("subsurface_radius", subsurface_radius);
    res.set_input("subsurface_anisotropy", anisotropic);
  }

  if (enabled(sheen)) {
    res.set_input("sheen", sheen);
    res.set_input("sheen_color", base_color.to_color3());
    res.set_input("sheen_roughness", roughness);
  }

  if (enabled(clearcoat)) {
    res.set_input("coat", clearcoat);
    res.set_input("coat_color", base_color.to_color3());
    res.set_input("coat_roughness", clearcoat_roughness);
    res.set_input("coat_IOR", ior);
    res.set_input("coat_anisotropy", anisotropic);
    res.set_input("coat_rotation", anisotropic_rotation);
    res.set_input("coat_normal", clearcoat_normal);
  }

  if (enabled(emission)) {
    res.set_input("emission", emission_strength);
    res.set_input("emission_color", emission);
  }

  return res;
}

}  // namespace blender::nodes::materialx
