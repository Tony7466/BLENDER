/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <filesystem>

#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/vt/array.h>
#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/imaging/hd/tokens.h>
#include <pxr/usd/usdLux/tokens.h>

#include "BKE_context.h"
#include "DNA_node_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_image.h"
#include "BKE_node.h"
#include "BKE_node_runtime.hh"
#include "NOD_shader.h"

#include "../utils.h"
#include "blender_scene_delegate.h"
#include "world.h"

/* TODO : add custom tftoken "transparency"? */

namespace blender::render::hydra {

WorldData::WorldData(BlenderSceneDelegate *scene_delegate, World *world, bContext *context)
    : IdData(scene_delegate, (ID *)world), context_(context)
{
  p_id_ = prim_id(scene_delegate);
  ID_LOG(2, "");
}

std::unique_ptr<WorldData> WorldData::create(BlenderSceneDelegate *scene_delegate,
                                             World *world,
                                             bContext *context)
{
  auto data = std::make_unique<WorldData>(scene_delegate, world, context);
  data->init();
  data->insert();
  return data;
}

pxr::SdfPath WorldData::prim_id(BlenderSceneDelegate *scene_delegate)
{
  return scene_delegate->GetDelegateID().AppendElementString("World");
}

void WorldData::init()
{
  ID_LOG(2, "");

  World *world = (World *)id_;
  data_.clear();

  data_[pxr::UsdLuxTokens->orientToStageUpAxis] = true;

  if (world->use_nodes) {
    /* TODO: Create nodes parsing system */

    bNode *output_node = ntreeShaderOutputNode(world->nodetree, SHD_OUTPUT_ALL);
    blender::Span<bNodeSocket *> input_sockets = output_node->input_sockets();
    bNodeSocket *input_socket = nullptr;

    for (auto socket : input_sockets) {
      if (STREQ(socket->name, "Surface")) {
        input_socket = socket;
        break;
      }
    }
    if (!input_socket) {
      return;
    }
    bNodeLink const *link = input_socket->directly_linked_links()[0];
    if (input_socket->directly_linked_links().is_empty()) {
      return;
    }

    bNode *input_node = link->fromnode;
    if (input_node->type != SH_NODE_BACKGROUND) {
      return;
    }

    bNodeSocket color_input = input_node->input_by_identifier("Color");
    bNodeSocket strength_input = input_node->input_by_identifier("Strength");

    float const *strength = strength_input.default_value_typed<float>();
    float const *color = color_input.default_value_typed<float>();
    data_[pxr::HdLightTokens->intensity] = strength[1];
    data_[pxr::HdLightTokens->exposure] = 1.0f;
    data_[pxr::HdLightTokens->color] = pxr::GfVec3f(color[0], color[1], color[2]);

    if (!color_input.directly_linked_links().is_empty()) {
      bNode *color_input_node = color_input.directly_linked_links()[0]->fromnode;
      if (color_input_node->type == SH_NODE_TEX_IMAGE) {
        NodeTexImage *tex = static_cast<NodeTexImage *>(color_input_node->storage);
        Image *image = (Image *)color_input_node->id;

        if (image) {
          Main *bmain = CTX_data_main(context_);
          Scene *scene = CTX_data_scene(context_);

          ReportList reports;
          ImageSaveOptions opts;
          opts.im_format.imtype = R_IMF_IMTYPE_PNG;

          std::string image_path = cache_image(bmain, scene, image, &tex->iuser, &opts, &reports);
          if (!image_path.empty()) {
            data_[pxr::HdLightTokens->textureFile] = pxr::SdfAssetPath(image_path, image_path);
          }
        }
      }
    }
  }
  else {
    data_[pxr::HdLightTokens->intensity] = 1.0f;
    data_[pxr::HdLightTokens->exposure] = world->exposure;
    data_[pxr::HdLightTokens->color] = pxr::GfVec3f(world->horr, world->horg, world->horb);
  }
}

void WorldData::insert()
{
  ID_LOG(2, "");
  scene_delegate_->GetRenderIndex().InsertSprim(
      pxr::HdPrimTypeTokens->domeLight, scene_delegate_, p_id_);
}

void WorldData::remove()
{
  CLOG_INFO(LOG_BSD, 2, "%s", id_->name);
  scene_delegate_->GetRenderIndex().RemoveSprim(pxr::HdPrimTypeTokens->domeLight, p_id_);
}

void WorldData::update()
{
  ID_LOG(2, "");
  init();
  scene_delegate_->GetRenderIndex().GetChangeTracker().MarkSprimDirty(p_id_,
                                                                      pxr::HdLight::AllDirty);
}

void WorldData::update(World *world)
{
  id_ = (ID *)world;
  update();
}

pxr::GfMatrix4d WorldData::transform()
{
  pxr::GfMatrix4d transform = pxr::GfMatrix4d(pxr::GfRotation(pxr::GfVec3d(1.0, 0.0, 0.0), -90),
                                              pxr::GfVec3d());

  /* TODO : do this check via RenderSettings*/
  if (scene_delegate_->GetRenderIndex().GetRenderDelegate()->GetRendererDisplayName() == "RPR") {
    transform *= pxr::GfMatrix4d(pxr::GfRotation(pxr::GfVec3d(1.0, 0.0, 0.0), -180),
                                 pxr::GfVec3d());
    transform *= pxr::GfMatrix4d(pxr::GfRotation(pxr::GfVec3d(0.0, 0.0, 1.0), 90.0),
                                 pxr::GfVec3d());
  }
  return transform;
}

pxr::VtValue WorldData::get_data(pxr::TfToken const &key) const
{
  pxr::VtValue ret;
  auto it = data_.find(key);
  if (it != data_.end()) {
    ret = it->second;
  }
  return ret;
}

}  // namespace blender::render::hydra
