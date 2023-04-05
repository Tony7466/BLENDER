/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <boost/algorithm/string/predicate.hpp>

#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/tokens.h>
#include <pxr/usd/usdLux/tokens.h>

#include "BKE_light.h"
#include "DNA_light_types.h"

#include "blender_scene_delegate.h"
#include "light.h"

namespace blender::render::hydra {

LightData::LightData(BlenderSceneDelegate *scene_delegate, Object *object)
    : ObjectData(scene_delegate, object)
{
  CLOG_INFO(LOG_BSD, 2, "%s id=%s", id->name, p_id.GetText());
}

void LightData::init()
{
  CLOG_INFO(LOG_BSD, 2, "%s", id->name);

  Light *light = (Light *)((Object *)id)->data;
  data.clear();

  float intensity = light->energy;
  if (scene_delegate->engine_type == BlenderSceneDelegate::EngineType::PREVIEW) {
    intensity *= 0.001;
  }
  data[pxr::HdLightTokens->intensity] = intensity;

  data[pxr::HdLightTokens->color] = pxr::GfVec3f(light->r, light->g, light->b);

  switch (light->type) {
    case LA_LOCAL:
      data[pxr::HdLightTokens->radius] = light->area_size / 2;
      break;

    case LA_SUN:
      data[pxr::HdLightTokens->angle] = light->sun_angle * 180.0 / M_PI;
      break;

    case LA_SPOT:
      data[pxr::HdLightTokens->shapingConeAngle] = light->spotsize / 2;
      data[pxr::HdLightTokens->shapingConeSoftness] = light->spotblend;
      data[pxr::UsdLuxTokens->treatAsPoint] = true;
      break;

    case LA_AREA:
      switch (light->area_shape) {
        case LA_AREA_SQUARE:
          data[pxr::HdLightTokens->width] = light->area_size;
          data[pxr::HdLightTokens->height] = light->area_size;
          break;
        case LA_AREA_RECT:
          data[pxr::HdLightTokens->width] = light->area_size;
          data[pxr::HdLightTokens->height] = light->area_sizey;
          break;

        case LA_AREA_DISK:
          data[pxr::HdLightTokens->radius] = light->area_size / 2;
          break;

        case LA_AREA_ELLIPSE:
          data[pxr::HdLightTokens->radius] = (light->area_size + light->area_sizey) / 4;
          break;

        default:
          break;
      }
      data[pxr::HdLightTokens->normalize] = true;
      break;

    default:
      break;
  }

  /* TODO: temporary value, it should be delivered through Python UI */
  data[pxr::HdLightTokens->exposure] = 1.0f;
}

pxr::TfToken LightData::prim_type()
{
  Light *light = (Light *)((Object *)id)->data;
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

pxr::VtValue LightData::get_data(pxr::TfToken const &key) const
{
  pxr::VtValue ret;
  auto it = data.find(key);
  if (it != data.end()) {
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

bool LightData::update_visibility(View3D *view3d)
{
  bool ret = ObjectData::update_visibility(view3d);
  if (ret) {
    scene_delegate->GetRenderIndex().GetChangeTracker().MarkSprimDirty(p_id,
                                                                       pxr::HdLight::DirtyParams);
  }
  return ret;
}

void LightData::insert()
{
  CLOG_INFO(LOG_BSD, 2, "%s", id->name);
  scene_delegate->GetRenderIndex().InsertSprim(prim_type(), scene_delegate, p_id);
}

void LightData::remove()
{
  CLOG_INFO(LOG_BSD, 2, "%s", id->name);
  scene_delegate->GetRenderIndex().RemoveSprim(prim_type(), p_id);
}

void LightData::update()
{
  /* TODO: prim_type was changed we have to do remove..add light */

  CLOG_INFO(LOG_BSD, 2, "%s", id->name);

  pxr::HdDirtyBits bits = pxr::HdLight::Clean;
  if (id->recalc & ID_RECALC_GEOMETRY) {
    init();
    bits = pxr::HdLight::AllDirty;
  }
  else if (id->recalc & ID_RECALC_TRANSFORM) {
    bits = pxr::HdLight::DirtyTransform;
  }
  scene_delegate->GetRenderIndex().GetChangeTracker().MarkSprimDirty(p_id, bits);
}

}  // namespace blender::render::hydra
