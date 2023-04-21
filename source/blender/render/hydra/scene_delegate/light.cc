/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <boost/algorithm/string/predicate.hpp>

#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/tokens.h>
#include <pxr/usd/usdLux/tokens.h>

#include "DNA_light_types.h"

#include "blender_scene_delegate.h"
#include "light.h"

namespace blender::render::hydra {

LightData::LightData(BlenderSceneDelegate *scene_delegate,
                     Object *object,
                     pxr::SdfPath const &prim_id)
    : ObjectData(scene_delegate, object, prim_id)
{
}

void LightData::init()
{
  ID_LOG(2, "");

  Light *light = (Light *)((Object *)id)->data;
  data_.clear();

  float intensity = light->energy;
  if (scene_delegate_->engine_type == BlenderSceneDelegate::EngineType::PREVIEW) {
    intensity *= 0.001;
  }
  data_[pxr::HdLightTokens->intensity] = intensity;

  data_[pxr::HdLightTokens->color] = pxr::GfVec3f(light->r, light->g, light->b);

  switch (light->type) {
    case LA_LOCAL:
      data_[pxr::HdLightTokens->radius] = light->area_size / 2;
      break;

    case LA_SUN:
      data_[pxr::HdLightTokens->angle] = light->sun_angle * 180.0 / M_PI;
      break;

    case LA_SPOT:
      data_[pxr::HdLightTokens->shapingConeAngle] = light->spotsize / 2;
      data_[pxr::HdLightTokens->shapingConeSoftness] = light->spotblend;
      data_[pxr::UsdLuxTokens->treatAsPoint] = true;
      break;

    case LA_AREA:
      switch (light->area_shape) {
        case LA_AREA_SQUARE:
          data_[pxr::HdLightTokens->width] = light->area_size;
          data_[pxr::HdLightTokens->height] = light->area_size;
          break;
        case LA_AREA_RECT:
          data_[pxr::HdLightTokens->width] = light->area_size;
          data_[pxr::HdLightTokens->height] = light->area_sizey;
          break;

        case LA_AREA_DISK:
          data_[pxr::HdLightTokens->radius] = light->area_size / 2;
          break;

        case LA_AREA_ELLIPSE:
          data_[pxr::HdLightTokens->radius] = (light->area_size + light->area_sizey) / 4;
          break;

        default:
          break;
      }
      data_[pxr::HdLightTokens->normalize] = true;
      break;

    default:
      break;
  }

  prim_type_ = prim_type(light);

  /* TODO: temporary value, it should be delivered through Python UI */
  data_[pxr::HdLightTokens->exposure] = 1.0f;

  set_transform_to_object();
}

void LightData::insert()
{
  ID_LOG(2, "");
  scene_delegate_->GetRenderIndex().InsertSprim(prim_type_, scene_delegate_, prim_id);
}

void LightData::remove()
{
  CLOG_INFO(LOG_BSD, 2, "%s", prim_id.GetText());
  scene_delegate_->GetRenderIndex().RemoveSprim(prim_type_, prim_id);
}

void LightData::update()
{
  ID_LOG(2, "");

  Light *light = (Light *)((Object *)id)->data;
  if (prim_type(light) != prim_type_) {
    remove();
    init();
    insert();
    return;
  }

  pxr::HdDirtyBits bits = pxr::HdLight::Clean;
  if (id->recalc & ID_RECALC_GEOMETRY) {
    init();
    bits = pxr::HdLight::AllDirty;
  }
  else if (id->recalc & ID_RECALC_TRANSFORM) {
    set_transform_to_object();
    bits = pxr::HdLight::DirtyTransform;
  }
  scene_delegate_->GetRenderIndex().GetChangeTracker().MarkSprimDirty(prim_id, bits);
}

pxr::VtValue LightData::get_data(pxr::TfToken const &key) const
{
  pxr::VtValue ret;
  auto it = data_.find(key);
  if (it != data_.end()) {
    ret = it->second;
  }
  else {
    std::string n = key.GetString();
    if (boost::algorithm::contains(n, "object:visibility:")) {
      if (boost::algorithm::ends_with(n, "camera") || boost::algorithm::ends_with(n, "shadow")) {
        ret = false;
      }
      else {
        ret = true;
      }
    }
  }
  return ret;
}

bool LightData::update_visibility()
{
  bool ret = ObjectData::update_visibility();
  if (ret) {
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkSprimDirty(prim_id,
                                                                        pxr::HdLight::DirtyParams);
  }
  return ret;
}

pxr::TfToken LightData::prim_type(Light *light)
{
  pxr::TfToken ret;
  switch (light->type) {
    case LA_LOCAL:
    case LA_SPOT:
      ret = pxr::HdPrimTypeTokens->sphereLight;
      break;

    case LA_SUN:
      ret = pxr::HdPrimTypeTokens->distantLight;
      break;

    case LA_AREA:
      switch (light->area_shape) {
        case LA_AREA_SQUARE:
        case LA_AREA_RECT:
          ret = pxr::HdPrimTypeTokens->rectLight;
          break;

        case LA_AREA_DISK:
        case LA_AREA_ELLIPSE:
          ret = pxr::HdPrimTypeTokens->diskLight;
          break;

        default:
          ret = pxr::HdPrimTypeTokens->rectLight;
      }
      break;

    default:
      ret = pxr::HdPrimTypeTokens->sphereLight;
  }
  return ret;
}

}  // namespace blender::render::hydra
