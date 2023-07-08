/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <boost/algorithm/string/predicate.hpp>

#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/tokens.h>
#include <pxr/usd/usdLux/tokens.h>

#include "BLI_math_rotation.h"
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
  ID_LOG(1, "");

  Light *light = (Light *)((Object *)id)->data;
  data_.clear();

  float intensity = light->energy;
  data_[pxr::HdLightTokens->color] = pxr::GfVec3f(light->r, light->g, light->b);

  switch (light->type) {
    case LA_LOCAL:
      if (light->radius <= FLT_EPSILON) {
        /* extremely small object should be considered as point */
        data_[pxr::UsdLuxTokens->treatAsPoint] = true;
      }
      else {
        data_[pxr::HdLightTokens->radius] = light->radius;
        data_[pxr::HdLightTokens->normalize] = true;
      }
      intensity /= 40.0f; /* coefficient approximated to follow Cycles results */
      break;

    case LA_SUN:
      data_[pxr::HdLightTokens->angle] = RAD2DEGF(light->sun_angle * 0.5f);
      break;

    case LA_SPOT:
      data_[pxr::UsdLuxTokens->inputsShapingConeAngle] = RAD2DEGF(light->spotsize * 0.5f);
      data_[pxr::UsdLuxTokens->inputsShapingConeSoftness] = light->spotblend;
      data_[pxr::UsdLuxTokens->treatAsPoint] = true;
      intensity /= 10.0f; /* coefficient approximated to follow Cycles results */
      break;

    case LA_AREA:
      switch (light->area_shape) {
        case LA_AREA_SQUARE:
          data_[pxr::HdLightTokens->width] = light->area_size;
          data_[pxr::HdLightTokens->height] = light->area_size;
          intensity /= 4.0f; /* coefficient approximated to follow Cycles results */
          break;
        case LA_AREA_RECT:
          data_[pxr::HdLightTokens->width] = light->area_size;
          data_[pxr::HdLightTokens->height] = light->area_sizey;
          intensity /= 4.0f; /* coefficient approximated to follow Cycles results */
          break;

        case LA_AREA_DISK:
          data_[pxr::HdLightTokens->radius] = light->area_size / 2.0f;
          intensity /= 16.0f; /* coefficient approximated to follow Cycles results */
          break;

        case LA_AREA_ELLIPSE:
          data_[pxr::HdLightTokens->radius] = (light->area_size + light->area_sizey) / 4.0f;
          intensity /= 16.0f; /* coefficient approximated to follow Cycles results */
          break;

        default:
          break;
      }
      data_[pxr::HdLightTokens->normalize] = true;
      break;

    default:
      break;
  }

  data_[pxr::HdLightTokens->intensity] = intensity;
  data_[pxr::HdLightTokens->exposure] = 0.0f;

  prim_type_ = prim_type(light);

  write_transform();
}

void LightData::insert()
{
  ID_LOG(1, "");
  scene_delegate_->GetRenderIndex().InsertSprim(prim_type_, scene_delegate_, prim_id);
}

void LightData::remove()
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s", prim_id.GetText());
  scene_delegate_->GetRenderIndex().RemoveSprim(prim_type_, prim_id);
}

void LightData::update()
{
  Object *object = (Object *)id;
  Light *light = (Light *)object->data;
  pxr::HdDirtyBits bits = pxr::HdLight::Clean;
  if (id->recalc & ID_RECALC_GEOMETRY || light->id.recalc & ID_RECALC_GEOMETRY) {
    if (prim_type(light) != prim_type_) {
      remove();
      init();
      insert();
      return;
    }
    init();
    bits = pxr::HdLight::AllDirty;
  }
  else if (id->recalc & ID_RECALC_TRANSFORM) {
    write_transform();
    bits = pxr::HdLight::DirtyTransform;
  }
  if (bits != pxr::HdChangeTracker::Clean) {
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkSprimDirty(prim_id, bits);
    ID_LOG(1, "");
  }
}

pxr::VtValue LightData::get_data(pxr::TfToken const &key) const
{
  ID_LOG(3, "%s", key.GetText());
  auto it = data_.find(key);
  if (it != data_.end()) {
    return pxr::VtValue(it->second);
  }

  pxr::VtValue *ret_ptr = scene_delegate_->settings.render_tokens.lookup_ptr(key);
  if (ret_ptr) {
    return *ret_ptr;
  }

  return pxr::VtValue();
}

bool LightData::update_visibility()
{
  bool ret = ObjectData::update_visibility();
  if (ret) {
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkSprimDirty(prim_id,
                                                                        pxr::HdLight::DirtyParams);
    ID_LOG(1, "");
  }
  return ret;
}

pxr::TfToken LightData::prim_type(Light *light)
{
  switch (light->type) {
    case LA_LOCAL:
    case LA_SPOT:
      return pxr::TfToken(pxr::HdPrimTypeTokens->sphereLight);

    case LA_SUN:
      return pxr::TfToken(pxr::HdPrimTypeTokens->distantLight);

    case LA_AREA:
      switch (light->area_shape) {
        case LA_AREA_SQUARE:
        case LA_AREA_RECT:
          return pxr::TfToken(pxr::HdPrimTypeTokens->rectLight);

        case LA_AREA_DISK:
        case LA_AREA_ELLIPSE:
          return pxr::TfToken(pxr::HdPrimTypeTokens->diskLight);

        default:
          return pxr::TfToken(pxr::HdPrimTypeTokens->rectLight);
      }
      break;

    default:
      return pxr::TfToken(pxr::HdPrimTypeTokens->sphereLight);
  }
}

}  // namespace blender::render::hydra
