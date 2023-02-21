/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <boost/algorithm/string/predicate.hpp>

#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/tokens.h>
#include <pxr/usd/usdLux/tokens.h>

#include "glog/logging.h"

#include "BKE_light.h"
#include "DNA_light_types.h"

#include "light.h"

using namespace pxr;
using namespace boost::algorithm;

namespace blender::render::hydra {

LightData::LightData(pxr::HdSceneDelegate *scene_delegate, Object *object)
  : ObjectData(scene_delegate, object)
{
  Light *light = (Light *)((Object *)id)->data;

  data[HdLightTokens->intensity] = light->energy;
  data[HdLightTokens->color] = GfVec3f(light->r, light->g, light->b);

  switch (light->type) {
    case LA_LOCAL:
      data[HdLightTokens->radius] = light->area_size / 2;
      break;

    case LA_SUN:
      data[HdLightTokens->angle] = light->sun_angle * 180.0 / M_PI;
      break;

    case LA_SPOT:
      data[HdLightTokens->shapingConeAngle] = light->spotsize / 2;
      data[HdLightTokens->shapingConeSoftness] = light->spotblend;
      data[UsdLuxTokens->treatAsPoint] = true;
      break;

    case LA_AREA:
      switch (light->area_shape) {
        case LA_AREA_SQUARE:
          data[HdLightTokens->width] = light->area_size;
          data[HdLightTokens->height] = light->area_size;
          break;
        case LA_AREA_RECT:
          data[HdLightTokens->width] = light->area_size;
          data[HdLightTokens->height] = light->area_sizey;
          break;

        case LA_AREA_DISK:
          data[HdLightTokens->radius] = light->area_size / 2;
          break;

        case LA_AREA_ELLIPSE:
          data[HdLightTokens->radius] = (light->area_size + light->area_sizey) / 4;
          break;

        default:
          break;
      }
      data[HdLightTokens->normalize] = true;
      break;

    default:
      break;
  }

  /* TODO: temporary value, it should be delivered through Python UI */
  data[HdLightTokens->exposure] = 1.0f;
}

pxr::TfToken LightData::prim_type()
{
  Light *light = (Light *)((Object *)id)->data;
  TfToken ret;
  switch (light->type) {
    case LA_LOCAL:
    case LA_SPOT:
      ret = HdPrimTypeTokens->sphereLight;
      break;

    case LA_SUN:
      ret = HdPrimTypeTokens->distantLight;
      break;

    case LA_AREA:
      switch (light->area_shape) {
        case LA_AREA_SQUARE:
        case LA_AREA_RECT:
          ret = HdPrimTypeTokens->rectLight;
          break;

        case LA_AREA_DISK:
        case LA_AREA_ELLIPSE:
          ret = HdPrimTypeTokens->diskLight;
          break;

        default:
          ret = HdPrimTypeTokens->rectLight;
      }
      break;

    default:
      ret = HdPrimTypeTokens->sphereLight;
  }
  return ret;
}

VtValue LightData::get_data(TfToken const &key)
{
  LOG(INFO) << "Get data light: " << name() << " [" << key.GetString() << "]";

  VtValue ret;
  auto it = data.find(key);
  if (it != data.end()) {
    ret = it->second;
  }
  else {
    std::string n = key.GetString();
    if (contains(n, "object:visibility:")) {
      if (ends_with(n, "camera") || ends_with(n, "shadow")) {
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
  SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  scene_delegate->GetRenderIndex().InsertSprim(prim_type(), scene_delegate, p_id);
  LOG(INFO) << "Add light: " << name() << " id=" << p_id.GetAsString();
}

void LightData::remove_prim()
{
  SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  scene_delegate->GetRenderIndex().RemoveSprim(prim_type(), p_id);
  LOG(INFO) << "Remove light: " << name();
}

void LightData::mark_prim_dirty(DirtyBits dirty_bits)
{
  /* TODO: prim_type was changed we have to do remove..add light */

  HdDirtyBits bits = HdLight::Clean;
  switch (dirty_bits) {
    case DirtyBits::DirtyTransform:
      bits = HdLight::DirtyTransform;
      break;
    case DirtyBits::DirtyVisibility:
      bits = HdLight::DirtyParams;
      break;
    case DirtyBits::AllDirty:
      bits = HdLight::AllDirty;
      break;
    default:
      break;
  }
  SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  scene_delegate->GetRenderIndex().GetChangeTracker().MarkSprimDirty(p_id, bits);
  LOG(INFO) << "Update light: " << name() << " [" << (int)dirty_bits << "]";
}

} // namespace blender::render::hydra
