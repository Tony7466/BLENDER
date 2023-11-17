/* SPDX-FileCopyrightText: 2023 NVIDIA Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd_reader_instance.h"

#include "BKE_object.hh"
#include "DNA_object_types.h"

#include <iostream>

namespace blender::io::usd {

USDInstanceReader::USDInstanceReader(const pxr::UsdPrim &prim,
                                     const USDImportParams &import_params,
                                     const ImportSettings &settings)
    : USDXformReader(prim, import_params, settings)
{
}

bool USDInstanceReader::valid() const
{
  return prim_.IsValid() && prim_.IsInstance();
}

void USDInstanceReader::create_object(Main *bmain, const double /* motionSampleTime */)
{
  this->object_ = BKE_object_add_only_object(bmain, OB_EMPTY, name_.c_str());
  this->object_->data = nullptr;
  this->object_->transflag |= OB_DUPLICOLLECTION;
}

void USDInstanceReader::set_instance_collection(Collection *coll)
{
  if (this->object_) {
    this->object_->instance_collection = coll;
  }
}

pxr::SdfPath USDInstanceReader::proto_path() const
{
  if (pxr::UsdPrim proto = prim_.GetPrototype()) {
    return proto.GetPath();
  }

  return pxr::SdfPath();
}

}  // namespace blender::io::usd
