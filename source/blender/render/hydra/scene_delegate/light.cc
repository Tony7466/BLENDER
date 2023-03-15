/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <boost/algorithm/string/predicate.hpp>

#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/tokens.h>
#include <pxr/usd/usdLux/tokens.h>

#include "glog/logging.h"

#include "BKE_light.h"
#include "DNA_light_types.h"

#include "blender_scene_delegate.h"
#include "light.h"

namespace blender::render::hydra {

LightData::LightData(BlenderSceneDelegate *scene_delegate, Object *object)
    : ObjectData(scene_delegate, object)
{
  Light *light = (Light *)((Object *)id)->data;

  data[pxr::HdLightTokens->intensity] = scene_delegate->engine_type ==
                                                BlenderSceneDelegate::EngineType::PREVIEW ?
                                            light->energy / 1000 :
                                            light->energy;

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

pxr::VtValue LightData::get_data(pxr::TfToken const &key)
{
  LOG(INFO) << "Get data light: " << name() << " [" << key.GetString() << "]";

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

void LightData::insert_prim()
{
  pxr::SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  scene_delegate->GetRenderIndex().InsertSprim(prim_type(), scene_delegate, p_id);
  LOG(INFO) << "Add light: " << name() << " id=" << p_id.GetAsString();
}

void LightData::remove_prim()
{
  pxr::SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  scene_delegate->GetRenderIndex().RemoveSprim(prim_type(), p_id);
  LOG(INFO) << "Remove light: " << name();
}

void LightData::mark_prim_dirty(DirtyBits dirty_bits)
{
  /* TODO: prim_type was changed we have to do remove..add light */

  pxr::HdDirtyBits bits = pxr::HdLight::Clean;
  switch (dirty_bits) {
    case DirtyBits::DIRTY_TRANSFORM:
      bits = pxr::HdLight::DirtyTransform;
      break;
    case DirtyBits::DIRTY_VISIBILITY:
      bits = pxr::HdLight::DirtyParams;
      break;
    case DirtyBits::ALL_DIRTY:
      bits = pxr::HdLight::AllDirty;
      break;
    default:
      break;
  }
  pxr::SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  scene_delegate->GetRenderIndex().GetChangeTracker().MarkSprimDirty(p_id, bits);
  LOG(INFO) << "Update light: " << name() << " [" << (int)dirty_bits << "]";
}

}  // namespace blender::render::hydra
