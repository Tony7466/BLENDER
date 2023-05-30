/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2020-2022 Blender Foundation. */

#ifndef __BVH_MULTI_H__
#define __BVH_MULTI_H__

#include "bvh/bvh.h"
#include "bvh/params.h"

#include "util/unique_ptr.h"
#include "util/vector.h"

CCL_NAMESPACE_BEGIN

class BVHMulti : public BVH {
 public:
  vector<unique_ptr<BVH>> sub_bvhs;

  virtual BVH *get_device_bvh(const Device *device) override;
  virtual void set_device_bvh(const Device *sub_device, BVH *bvh) override;

 protected:
  friend class BVH;
  BVHMulti(const BVHParams &params,
           const vector<Geometry *> &geometry,
           const vector<Object *> &objects,
           const Device *device);
  virtual ~BVHMulti();

  const Device *device;

  virtual void replace_geometry(const vector<Geometry *> &geometry,
                                const vector<Object *> &objects) override;
};

CCL_NAMESPACE_END

#endif /* __BVH_MULTI_H__ */
