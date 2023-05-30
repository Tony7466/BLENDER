/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2020-2022 Blender Foundation. */

#include "bvh/multi.h"
#include "device/device.h"
#include "util/foreach.h"

CCL_NAMESPACE_BEGIN

BVHMulti::BVHMulti(const BVHParams &params_,
                   const vector<Geometry *> &geometry_,
                   const vector<Object *> &objects_,
                   const Device *device_)
    : BVH(params_, geometry_, objects_), device(device_)
{
  sub_bvhs.resize(device->get_num_devices());
}

BVHMulti::~BVHMulti() {}

BVH *BVHMulti::get_device_bvh(const Device *subdevice)
{
  if (subdevice == device) {
    return this;
  }
  else {
    const int id = device->device_number(subdevice);
    assert(id != -1 && id < sub_bvhs.size());
    return sub_bvhs[id].get();
  }
}

void BVHMulti::set_device_bvh(const Device *subdevice, BVH *bvh)
{
  int id = device->device_number(subdevice);
  sub_bvhs[id].reset(bvh);
};

void BVHMulti::replace_geometry(const vector<Geometry *> &geometry,
                                const vector<Object *> &objects)
{
  foreach (auto &bvh, sub_bvhs) {
    bvh->replace_geometry(geometry, objects);
  }
}

CCL_NAMESPACE_END
