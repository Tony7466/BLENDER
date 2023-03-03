/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <filesystem>

#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/rotation.h>
#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/tokens.h>
#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/usd/usdLux/tokens.h>

#include "BKE_context.h"
#include "DNA_node_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_node.h"
#include "BKE_node_runtime.hh"
#include "BKE_image.h"
#include "NOD_shader.h"

#include "glog/logging.h"

#include "blenderSceneDelegate.h"
#include "world.h"
#include "../utils.h"

/* TODO : add custom tftoken "transparency"? */

using namespace pxr;

namespace blender::render::hydra {

std::unique_ptr<WorldData> WorldData::init(BlenderSceneDelegate *scene_delegate,
                                           World *world, bContext *context)
{
  return std::make_unique<WorldData>(scene_delegate, world, context);
}

SdfPath WorldData::prim_id(BlenderSceneDelegate *scene_delegate)
{
  return scene_delegate->GetDelegateID().AppendElementString("World");
}

WorldData::WorldData(BlenderSceneDelegate *scene_delegate, World *world, bContext *context)
  : IdData(scene_delegate, (ID *)world)
{
  data[UsdLuxTokens->orientToStageUpAxis] = true;

  if (world->use_nodes) {
    /* TODO: Create nodes parsing system */

    bNode *output_node = ntreeShaderOutputNode(world->nodetree, SHD_OUTPUT_ALL);
    bNodeSocket input_socket = output_node->input_by_identifier("Surface");
    bNodeLink const *link = input_socket.directly_linked_links()[0];
    if (input_socket.directly_linked_links().is_empty()) {
      return;
    }

    bNode *input_node = link->fromnode;

    bNodeSocket color_input = input_node->input_by_identifier("Color");
    bNodeSocket strength_input = input_node->input_by_identifier("Strength");

    float const *strength = strength_input.default_value_typed<float>();
    float const *color = color_input.default_value_typed<float>();
    data[HdLightTokens->intensity] = strength[1];
    data[HdLightTokens->exposure] = 1.0f;
    data[HdLightTokens->color] = GfVec3f(color[0], color[1], color[2]);

    if (!color_input.directly_linked_links().is_empty()) {
      bNode *color_input_node = color_input.directly_linked_links()[0]->fromnode;
      if (color_input_node->type == SH_NODE_TEX_IMAGE) {
        NodeTexImage *tex = static_cast<NodeTexImage *>(color_input_node->storage);
        Image *image = (Image *)color_input_node->id;

        if (image) {
          Main *bmain = CTX_data_main(context);
          Scene *scene = CTX_data_scene(context);

          ReportList reports;
          ImageSaveOptions opts;
          opts.im_format.imtype = R_IMF_IMTYPE_PNG;

          std::string image_path = cache_image(bmain, scene, image, &tex->iuser, &opts, &reports);
          if (!image_path.empty()) {
            data[HdLightTokens->textureFile] = SdfAssetPath(image_path, image_path);
          }
        }
      }
    }
  }
  else {
    data[HdLightTokens->intensity] = 1.0f;
    data[HdLightTokens->exposure] = world->exposure;
    data[HdLightTokens->color] = GfVec3f(world->horr, world->horg, world->horb);
  }
}

GfMatrix4d WorldData::transform()
{
  GfMatrix4d transform = GfMatrix4d(GfRotation(GfVec3d(1.0, 0.0, 0.0), -90), GfVec3d());

  /* TODO : do this check via RenderSettings*/ 
  if (scene_delegate->GetRenderIndex().GetRenderDelegate()->GetRendererDisplayName() == "RPR") {
    transform *= GfMatrix4d(GfRotation(GfVec3d(1.0, 0.0, 0.0), -180), GfVec3d());
    transform *= GfMatrix4d(GfRotation(GfVec3d(0.0, 0.0, 1.0), 90.0), GfVec3d());
  }
  return transform;
}

VtValue WorldData::get_data(TfToken const &key)
{
  VtValue ret;
  auto it = data.find(key);
  if (it != data.end()) {
    ret = it->second;
  }
  return ret;
}

void WorldData::insert_prim()
{
  SdfPath p_id = prim_id(scene_delegate);
  scene_delegate->GetRenderIndex().InsertSprim(HdPrimTypeTokens->domeLight, scene_delegate, p_id);
  LOG(INFO) << "Add World: id=" << p_id.GetAsString();
}

void WorldData::remove_prim()
{
  SdfPath p_id = prim_id(scene_delegate);
  scene_delegate->GetRenderIndex().RemoveSprim(HdPrimTypeTokens->domeLight, p_id);
  LOG(INFO) << "Remove World";
}

void WorldData::mark_prim_dirty(DirtyBits dirty_bits)
{
  HdDirtyBits bits = HdLight::Clean;
  switch (dirty_bits) {
    case DirtyBits::AllDirty:
      bits = HdLight::AllDirty;
      break;
    default:
      break;
  }
  SdfPath p_id = prim_id(scene_delegate);
  scene_delegate->GetRenderIndex().GetChangeTracker().MarkSprimDirty(p_id, bits);
  LOG(INFO) << "Update World";
}

}  // namespace blender::render::hydra
