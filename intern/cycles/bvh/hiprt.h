/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#ifdef WITH_HIPRT
#  ifndef __HIPRT_H__
#    define __HIPRT_H__

#    include "bvh/bvh.h"
#    include "bvh/params.h"

#    ifdef WITH_HIPRT
#      include "hiprt/hiprt_types.h"
#    endif

#    include "device/memory.h"

CCL_NAMESPACE_BEGIN

class BVHHIPRT : public BVH {
 public:
  friend class HIPDevice;

  bool is_tlas()
  {
    return params.top_level;
  }

  hiprtGeometry hiprt_geom;
  hiprtTriangleMeshPrimitive triangle_mesh;
  hiprtAABBListPrimitive custom_prim_aabb;
  hiprtGeometryBuildInput geom_input;

  vector<int2> custom_prim_info;  // x: prim_id, y: prim_type
  vector<float2> prims_time;

  // custom primitives

  device_vector<BoundBox> custom_primitive_bound;
  device_vector<int> triangle_index;
  device_vector<float> vertex_data;

 protected:
  friend class BVH;
  BVHHIPRT(const BVHParams &params,
           const vector<Geometry *> &geometry,
           const vector<Object *> &objects,
           Device *in_device);

  virtual ~BVHHIPRT();

 private:
  Device *device;
};

CCL_NAMESPACE_END

#  endif
#endif
