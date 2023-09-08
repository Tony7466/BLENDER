/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

#include "hydra/image.h"

#include "DEG_depsgraph_query.h"

namespace blender::nodes::materialx {

NodeItem TexImageNodeParser::compute()
{
  NodeItem res = val(MaterialX::Color4(1.0f, 0.0f, 1.0f, 1.0f));

  Image *image = (Image *)node_->id;
  if (image) {
    NodeTexImage *tex_image = static_cast<NodeTexImage *>(node_->storage);
    Scene *scene = DEG_get_input_scene(depsgraph_);
    Main *bmain = DEG_get_bmain(depsgraph_);

    /* TODO: What if Blender built without Hydra? Also io::hydra::cache_or_get_image_file contains
     * pretty general code, so could be moved from bf_usd project. */
    std::string image_path = io::hydra::cache_or_get_image_file(
        bmain, scene, image, &tex_image->iuser);

    NodeItem vector = get_input_link("Vector", NodeItem::Type::Vector2);
    if (!vector) {
      vector = texcoord_node();
    }
    /* TODO: add math to vector depending of tex_image->projection */

    std::string filtertype;
    switch (tex_image->interpolation) {
      case SHD_INTERP_LINEAR:
        filtertype = "linear";
        break;
      case SHD_INTERP_CLOSEST:
        filtertype = "closest";
        break;
      case SHD_INTERP_CUBIC:
      case SHD_INTERP_SMART:
        filtertype = "cubic";
        break;
      default:
        BLI_assert_unreachable();
    }
    std::string addressmode;
    switch (tex_image->extension) {
      case SHD_IMAGE_EXTENSION_REPEAT:
        addressmode = "periodic";
        break;
      case SHD_IMAGE_EXTENSION_EXTEND:
        addressmode = "clamp";
        break;
      case SHD_IMAGE_EXTENSION_CLIP:
        addressmode = "constant";
        break;
      case SHD_IMAGE_EXTENSION_MIRROR:
        addressmode = "mirror";
        break;
      default:
        BLI_assert_unreachable();
    }

    res = create_node("image", NodeItem::Type::Color4);
    res.set_input("file", image_path, NodeItem::Type::Filename);
    res.set_input("texcoord", vector);
    res.set_input("filtertype", val(filtertype));
    res.set_input("uaddressmode", val(addressmode));
    res.set_input("vaddressmode", val(addressmode));
  }

  if (STREQ(socket_out_->name, "Alpha")) {
    res = res.extract(3);
  }
  return res;
}

}  // namespace blender::nodes::materialx
