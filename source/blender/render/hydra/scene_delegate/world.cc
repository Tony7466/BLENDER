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

#include "BKE_node.h"
#include "BKE_node_runtime.hh"
#include "BKE_studiolight.h"
#include "BLI_math_rotation.h"
#include "BLI_path_util.h"
#include "NOD_shader.h"

#include "../engine.h"
#include "blender_scene_delegate.h"
#include "image.h"
#include "world.h"

/* TODO : add custom tftoken "transparency"? */

/* NOTE: opacity and blur aren't supported by USD */

namespace blender::render::hydra {

WorldData::WorldData(BlenderSceneDelegate *scene_delegate, pxr::SdfPath const &prim_id)
    : IdData(scene_delegate, nullptr, prim_id)
{
}

void WorldData::init()
{
  write_transform();

  data_.clear();
  data_[pxr::UsdLuxTokens->orientToStageUpAxis] = true;

  float intensity = 1.0f;
  float exposure = 1.0f;
  pxr::GfVec3f color(1.0f, 1.0f, 1.0f);
  pxr::SdfAssetPath texture_file;

  if (scene_delegate_->shading_settings.use_scene_world) {
    World *world = scene_delegate_->scene->world;
    CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s: %s", prim_id.GetText(), world->id.name);

    exposure = world->exposure;
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
      float const *input_color = color_input.default_value_typed<float>();
      intensity = strength[1];
      color = pxr::GfVec3f(input_color[0], input_color[1], input_color[2]);

      if (!color_input.directly_linked_links().is_empty()) {
        bNode *color_input_node = color_input.directly_linked_links()[0]->fromnode;
        if (color_input_node->type == SH_NODE_TEX_IMAGE) {
          NodeTexImage *tex = static_cast<NodeTexImage *>(color_input_node->storage);
          Image *image = (Image *)color_input_node->id;
          if (image) {
            std::string image_path = cache_or_get_image_file(
                image, scene_delegate_->context, &tex->iuser);
            if (!image_path.empty()) {
              texture_file = pxr::SdfAssetPath(image_path, image_path);
            }
          }
        }
      }
    }
    else {
      intensity = 1.0f;
      color = pxr::GfVec3f(world->horr, world->horg, world->horb);
    }

    if (texture_file.GetAssetPath().empty()) {
      float fill_color[4] = {color[0], color[1], color[2], 1.0f};
      std::string image_path = cache_image_color(fill_color);
      texture_file = pxr::SdfAssetPath(image_path, image_path);
    }
  }
  else {
    CLOG_INFO(LOG_RENDER_HYDRA_SCENE,
              1,
              "%s: studiolight: %s",
              prim_id.GetText(),
              scene_delegate_->shading_settings.studiolight_name.c_str());

    StudioLight *sl = BKE_studiolight_find(
        scene_delegate_->shading_settings.studiolight_name.c_str(),
        STUDIOLIGHT_ORIENTATIONS_MATERIAL_MODE);
    if (sl != NULL && sl->flag & STUDIOLIGHT_TYPE_WORLD) {
      texture_file = pxr::SdfAssetPath(sl->filepath, sl->filepath);
      transform *= pxr::GfMatrix4d(
          pxr::GfRotation(pxr::GfVec3d(0.0, 0.0, -1.0),
                          RAD2DEGF(scene_delegate_->shading_settings.studiolight_rotation)),
          pxr::GfVec3d());
      /* coefficient to follow Cycles result */
      intensity = scene_delegate_->shading_settings.studiolight_intensity / 2;
    }
  }

  data_[pxr::HdLightTokens->intensity] = intensity;
  data_[pxr::HdLightTokens->exposure] = exposure;
  data_[pxr::HdLightTokens->color] = color;
  data_[pxr::HdLightTokens->textureFile] = texture_file;
}

void WorldData::insert()
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s", prim_id.GetText());
  scene_delegate_->GetRenderIndex().InsertSprim(
      pxr::HdPrimTypeTokens->domeLight, scene_delegate_, prim_id);
}

void WorldData::remove()
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s", prim_id.GetText());
  scene_delegate_->GetRenderIndex().RemoveSprim(pxr::HdPrimTypeTokens->domeLight, prim_id);
}

void WorldData::update()
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s", prim_id.GetText());
  init();
  scene_delegate_->GetRenderIndex().GetChangeTracker().MarkSprimDirty(prim_id,
                                                                      pxr::HdLight::AllDirty);
}

pxr::VtValue WorldData::get_data(pxr::TfToken const &key) const
{
  auto it = data_.find(key);
  if (it != data_.end()) {
    CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s: %s", prim_id.GetText(), key.GetText());
    return pxr::VtValue(it->second);
  }
  return pxr::VtValue();
}

void WorldData::write_transform()
{
  transform = pxr::GfMatrix4d(pxr::GfRotation(pxr::GfVec3d(1.0, 0.0, 0.0), -90), pxr::GfVec3d());

  /* TODO : do this check via RenderSettings*/
  if (scene_delegate_->engine->render_delegate_name == "HdRprPlugin") {
    transform *= pxr::GfMatrix4d(pxr::GfRotation(pxr::GfVec3d(1.0, 0.0, 0.0), -180),
                                 pxr::GfVec3d());
    transform *= pxr::GfMatrix4d(pxr::GfRotation(pxr::GfVec3d(0.0, 0.0, 1.0), 90.0),
                                 pxr::GfVec3d());
  }
}

}  // namespace blender::render::hydra
