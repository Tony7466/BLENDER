/* SPDX-FileCopyrightText: 2021 Tangent Animation. All rights reserved.
 * SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Adapted from the Blender Alembic importer implementation. */

#include "usd_reader_prim.hh"
#include "usd_reader_utils.hh"

#include "usd.hh"

#include "WM_types.hh"

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>

#include <optional>

#include "BLI_assert.h"

namespace blender::io::usd {

void USDPrimReader::set_props(ID *id,
                              const pxr::UsdPrim &prim,
                              const bool use_parent,
                              const std::optional<double> motionSampleTime)
{
  eUSDAttrImportMode attr_import_mode = this->import_params_.attr_import_mode;

  if (attr_import_mode == USD_ATTR_IMPORT_NONE) {
    return;
  }

  const pxr::UsdPrim parent_prim = prim.GetParent();
  const pxr::UsdPrim check_prim = use_parent && (bool)parent_prim ? parent_prim : prim;

  set_id_props_from_prim(id, check_prim, attr_import_mode, motionSampleTime);
}

USDPrimReader::USDPrimReader(const pxr::UsdPrim &prim,
                             const USDImportParams &import_params,
                             const ImportSettings &settings)
    : name_(prim.GetName().GetString()),
      prim_path_(prim.GetPrimPath().GetString()),
      object_(nullptr),
      prim_(prim),
      import_params_(import_params),
      parent_reader_(nullptr),
      settings_(&settings),
      refcount_(0),
      is_in_instancer_proto_(false)
{
}

USDPrimReader::~USDPrimReader() = default;

const pxr::UsdPrim &USDPrimReader::prim() const
{
  return prim_;
}

void USDPrimReader::read_object_data(Main * /* bmain */, double motionSampleTime)
{
  if (!prim_ || !object_) {
    return;
  }

  ID *id = object_->data ? static_cast<ID *>(object_->data) : &object_->id;

  set_props(id, prim_, motionSampleTime);
}

Object *USDPrimReader::object() const
{
  return object_;
}

void USDPrimReader::object(Object *ob)
{
  object_ = ob;
}

bool USDPrimReader::valid() const
{
  return prim_.IsValid();
}

int USDPrimReader::refcount() const
{
  return refcount_;
}

void USDPrimReader::incref()
{
  refcount_++;
}

void USDPrimReader::decref()
{
  refcount_--;
  BLI_assert(refcount_ >= 0);
}

bool USDPrimReader::is_in_proto() const
{
  return prim_ && (prim_.IsInPrototype() || is_in_instancer_proto_);
}

}  // namespace blender::io::usd
