/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#ifdef __HIPRT__

/* TODO: Store self primitives by reference instead of copy? */

struct RayPayload {
  RaySelfPrimitives self;
  uint visibility;
  int prim_type;
  float ray_time;
};

struct ShadowPayload {
  RaySelfPrimitives self;
  uint visibility;
  int prim_type;
  float ray_time;
  int in_state;
  uint max_hits;
  uint num_hits;
  uint *r_num_recorded_hits;
  float *r_throughput;
};

struct LocalPayload {
  RaySelfPrimitives self;
  int prim_type;
  float ray_time;
  int local_object;
  uint max_hits;
  uint *lcg_state;
  LocalIntersection *local_isect;
};

#  define SET_HIPRT_RAY(RAY_RT, RAY) \
    RAY_RT.direction = RAY->D; \
    RAY_RT.origin = RAY->P; \
    RAY_RT.maxT = RAY->tmax; \
    RAY_RT.minT = RAY->tmin;

#  if defined(HIPRT_SHARED_STACK)
#    define GET_TRAVERSAL_STACK() \
      Stack stack(&kg->global_stack_buffer[0], \
                  HIPRT_THREAD_STACK_SIZE, \
                  kg->shared_stack, \
                  HIPRT_SHARED_STACK_SIZE);
#  else
#    define GET_TRAVERSAL_STACK()
#  endif

#  ifdef HIPRT_SHARED_STACK
#    define GET_TRAVERSAL_ANY_HIT(FUNCTION_TABLE, RAY_TYPE, RAY_TIME) \
      hiprtSceneTraversalAnyHitCustomStack<Stack> traversal(kernel_data.device_bvh, \
                                                            ray_hip, \
                                                            stack, \
                                                            visibility, \
                                                            hiprtTraversalHintDefault, \
                                                            &payload, \
                                                            kernel_params.FUNCTION_TABLE, \
                                                            RAY_TYPE, \
                                                            RAY_TIME);

#    define GET_TRAVERSAL_CLOSEST_HIT(FUNCTION_TABLE, RAY_TYPE, RAY_TIME) \
      hiprtSceneTraversalClosestCustomStack<Stack> traversal(kernel_data.device_bvh, \
                                                             ray_hip, \
                                                             stack, \
                                                             visibility, \
                                                             hiprtTraversalHintDefault, \
                                                             &payload, \
                                                             kernel_params.FUNCTION_TABLE, \
                                                             RAY_TYPE, \
                                                             RAY_TIME);
#  else
#    define GET_TRAVERSAL_ANY_HIT(FUNCTION_TABLE) \
      hiprtSceneTraversalAnyHit traversal(kernel_data.device_bvh, \
                                          ray_hip, \
                                          visibility, \
                                          FUNCTION_TABLE, \
                                          hiprtTraversalHintDefault, \
                                          &payload);
#    define GET_TRAVERSAL_CLOSEST_HIT(FUNCTION_TABLE) \
      hiprtSceneTraversalClosest traversal(kernel_data.device_bvh, \
                                           ray_hip, \
                                           visibility, \
                                           FUNCTION_TABLE, \
                                           hiprtTraversalHintDefault, \
                                           &payload);
#  endif

ccl_device_forceinline void set_intersect_point(hiprtHit &hit,
                                                ccl_private Intersection *isect)
{
  const int object_id = kernel_data_fetch(user_instance_id, hit.instanceID);

  isect->t = hit.t;
  isect->u = hit.uv.x;
  isect->v = hit.uv.y;
  isect->prim = hit.primID + kernel_data_fetch(object_prim_offset, object_id);
  isect->object = object_id;
  isect->type = kernel_data_fetch(objects, object_id).primitive_type;
}

/* Custom intersection functions. */

ccl_device_inline bool curve_custom_intersect(const hiprtRay &ccl_restrict ray,
                                              RayPayload *ccl_restrict payload,
                                              hiprtHit &ccl_restrict hit)

{
  /* Could also cast shadow payload to get the elements needed to do the intersection
   * no need to write a separate function for shadow intersection. */

  const int object_id = kernel_data_fetch(user_instance_id, hit.instanceID);
  const int2 data_offset = kernel_data_fetch(custom_prim_info_offset, object_id);
  /* data_offset.x: where the data (prim id, type) for the geometry of the current object begins
   * the prim_id that is in hiprtHit hit is local to the partciular geometry so we add the above
   * ofstream to map prim id in hiprtHit to the one compatible to what next stage expects.
   *
   * data_offset.y: the offset that has to be added to a local primitive to get the global
   * primitive id = kernel_data_fetch(object_prim_offset, object_id); */

  /* prim_info.x = curve_index,
   * primt_info.y = key_value. */
  const int2 prim_info = kernel_data_fetch(custom_prim_info, hit.primID + data_offset.x);

#  ifdef __SHADOW_LINKING__
  if (intersection_skip_shadow_link(nullptr, payload->self, object_id)) {
    /* Ignore hit - continue traversal. */
    return false;
  }
#  endif

  if (intersection_skip_self_shadow(payload->self, object_id, prim_info.x + data_offset.y)) {
    return false;
  }

  if ((prim_info.y & PRIMITIVE_MOTION) && kernel_data.bvh.use_bvh_steps) {
    const int time_offset = kernel_data_fetch(prim_time_offset, object_id);
    const float2 prims_time = kernel_data_fetch(prims_time, hit.primID + time_offset);
    if (payload->ray_time < prims_time.x || payload->ray_time > prims_time.y) {
      return false;
    }
  }

  Intersection isect;
  if (curve_intersect(nullptr,
                      &isect,
                      ray.origin,
                      ray.direction,
                      ray.minT,
                      ray.maxT,
                      object_id,
                      prim_info.x + data_offset.y,
                      payload->ray_time,
                      prim_info.y)) {
    hit.primID = isect.prim;
    hit.uv.x = isect.u;
    hit.uv.y = isect.v;
    hit.t = isect.t;
    payload->prim_type = isect.type;  /* packed_curve_type */
    return true;
  }

  return false;
}

ccl_device_inline bool motion_triangle_custom_intersect(const hiprtRay &ccl_restrict ray,
                                                        RayPayload *ccl_restrict payload,
                                                        hiprtHit &ccl_restrict hit)
{
  const int object_id = kernel_data_fetch(user_instance_id, hit.instanceID);
  const int2 data_offset = kernel_data_fetch(custom_prim_info_offset, object_id);
  const int prim_offset = kernel_data_fetch(object_prim_offset, object_id);
  const int prim_id_local = kernel_data_fetch(custom_prim_info, hit.primID + data_offset.x).x;

  if (intersection_skip_self_shadow(payload->self, object_id, prim_id_local + prim_offset)) {
    return false;
  }

  Intersection isect;
  if (motion_triangle_intersect(nullptr,
                                &isect,
                                ray.origin,
                                ray.direction,
                                ray.minT,
                                ray.maxT,
                                payload->ray_time,
                                payload->visibility,
                                object_id,
                                prim_id_local + prim_offset,
                                hit.instanceID)) {
    hit.primID = isect.prim;
    hit.uv.x = isect.u;
    hit.uv.y = isect.v;
    hit.t = isect.t;
    payload->prim_type = isect.type;
    return true;
  }

  return false;
}

ccl_device_inline bool motion_triangle_custom_local_intersect(const hiprtRay &ccl_restrict ray,
                                                              LocalPayload *ccl_restrict payload,
                                                              hiprtHit &ccl_restrict hit)
{
#  ifdef __OBJECT_MOTION__
  const int prim_offset = kernel_data_fetch(object_prim_offset, payload->local_object);
  const int2 data_offset = kernel_data_fetch(custom_prim_info_offset, payload->local_object);
  const int prim_id_local = kernel_data_fetch(custom_prim_info, hit.primID + data_offset.x).x;

  if (intersection_skip_self_local(payload->self, prim_id_local + prim_offset)) {
    return false;
  }

  if (motion_triangle_intersect_local(nullptr,
                                      payload->local_isect,
                                      ray.origin,
                                      ray.direction,
                                      payload->ray_time,
                                      payload->local_object,
                                      prim_id_local + prim_offset,
                                      prim_id_local,
                                      ray.minT,
                                      ray.maxT,
                                      payload->lcg_state,
                                      payload->max_hits)) {
    payload->prim_type = PRIMITIVE_MOTION_TRIANGLE;
    return true;
  }
#  endif

  return false;
}

ccl_device_inline bool motion_triangle_custom_volume_intersect(const hiprtRay &ccl_restrict ray,
                                                               RayPayload *ccl_restrict payload,
                                                               hiprtHit &ccl_restrict hit)
{
#  ifdef __OBJECT_MOTION__
  const int object_id = kernel_data_fetch(user_instance_id, hit.instanceID);

  if (!(kernel_data_fetch(object_flag, object_id) & SD_OBJECT_HAS_VOLUME)) {
    return false;
  }

  const int2 data_offset = kernel_data_fetch(custom_prim_info_offset, object_id);
  const int prim_offset = kernel_data_fetch(object_prim_offset, object_id);
  const int prim_id_local = kernel_data_fetch(custom_prim_info, hit.primID + data_offset.x).x;

  if (intersection_skip_self_shadow(payload->self, object_id, prim_id_local + prim_offset)) {
    return false;
  }

  Intersection isect;
  if (motion_triangle_intersect(nullptr,
                                &isect,
                                ray.origin,
                                ray.direction,
                                ray.minT,
                                ray.maxT,
                                payload->ray_time,
                                payload->visibility,
                                object_id,
                                prim_id_local + prim_offset,
                                prim_id_local)) {
    hit.primID = isect.prim;
    hit.uv.x = isect.u;
    hit.uv.y = isect.v;
    hit.t = isect.t;
    payload->prim_type = isect.type;
    return true;
  }
#  endif

  return false;
}

ccl_device_inline bool point_custom_intersect(const hiprtRay &ccl_restrict ray,
                                              RayPayload *ccl_restrict payload,
                                              hiprtHit &ccl_restrict hit)
{
#  if defined(__POINTCLOUD__) && 0
  const int object_id = kernel_data_fetch(user_instance_id, hit.instanceID);
  const int2 data_offset = kernel_data_fetch(custom_prim_info_offset, object_id);
  const int prim_offset = kernel_data_fetch(object_prim_offset, object_id);

  /* prim_info.x is prim_id_local,
   * prim_info.y is type. */
  const int2 prim_info = kernel_data_fetch(custom_prim_info, hit.primID + data_offset.x);

#    ifdef __SHADOW_LINKING__
  if (intersection_skip_shadow_link(nullptr, payload->self, object_id)) {
    /* Ignore hit - continue traversal. */
    return false;
  }
#    endif

  if (intersection_skip_self_shadow(payload->self, object_id, prim_info.x + prim_offset)) {
    return false;
  }

  if ((prim_info.y & PRIMITIVE_MOTION_POINT) && kernel_data.bvh.use_bvh_steps) {
    const int time_offset = kernel_data_fetch(prim_time_offset, object_id);
    const float2 prims_time = kernel_data_fetch(prims_time, hit.primID + time_offset);
    if (payload->ray_time < prims_time.x || payload->ray_time > prims_time.y) {
      return false;
    }
  }

  Intersection isect;
  if (point_intersect(nullptr,
                      &isect,
                      ray.origin,
                      ray.direction,
                      ray.minT,
                      ray.maxT,
                      object_id,
                      prim_info.x + prim_offset,
                      payload->ray_time,
                      prim_info.y))
  {
    hit.primID = isect.prim;
    hit.uv.x = isect.u;
    hit.uv.y = isect.v;
    hit.t = isect.t;
    payload->prim_type = isect.type;
    return true;
  }
#  endif

  return false;
}

/* Intersection filters. */

ccl_device_inline bool closest_intersection_filter(const hiprtRay &ccl_restrict ray,
                                                   RayPayload *ccl_restrict payload,
                                                   const hiprtHit &ccl_restrict hit)
{
  const int object_id = kernel_data_fetch(user_instance_id, hit.instanceID);
  const int prim_offset = kernel_data_fetch(object_prim_offset, object_id);

#  ifdef __SHADOW_LINKING__
  if (intersection_skip_shadow_link(nullptr, payload->self, object_id)) {
    /* Ignore hit - continue traversal. */
    return true;
  }
#  endif

  return intersection_skip_self_shadow(payload->self, object_id, hit.primID + prim_offset);
}

ccl_device_inline bool shadow_intersection_filter(const hiprtRay &ccl_restrict ray,
                                                  ShadowPayload *ccl_restrict payload,
                                                  const hiprtHit &ccl_restrict hit)

{
  const int object_id = kernel_data_fetch(user_instance_id, hit.instanceID);

#  ifdef __SHADOW_LINKING__
  if (intersection_skip_shadow_link(nullptr, payload->self, object_id)) {
    /* Ignore hit - continue traversal. */
    return true;
  }
#  endif

#  ifdef __VISIBILITY_FLAG__
  if ((kernel_data_fetch(objects, object_id).visibility & payload->visibility) == 0) {
    return true;  /* No hit - continue traversal. */
  }
#  endif

  const int prim_offset = kernel_data_fetch(object_prim_offset, object_id);
  if (intersection_skip_self_shadow(payload->self, object_id, hit.primID + prim_offset)) {
    return true;  /* No hit -continue traversal. */
  }

#  ifdef __TRANSPARENT_SHADOWS__
  const int type = kernel_data_fetch(objects, object_id).primitive_type;
  if (payload->num_hits >= payload->max_hits ||
      !(intersection_get_shader_flags(nullptr, hit.primID + prim_offset, type) & SD_HAS_TRANSPARENT_SHADOW))
  {
    return false;
  }

  uint record_index = *payload->r_num_recorded_hits;

  ++payload->num_hits;
  ++*(payload->r_num_recorded_hits);

  const uint max_record_hits = min(payload->max_hits, INTEGRATOR_SHADOW_ISECT_SIZE);
  if (record_index >= max_record_hits) {
    float max_recorded_t = INTEGRATOR_STATE_ARRAY(payload->in_state, shadow_isect, 0, t);
    uint max_recorded_hit = 0;

    for (int i = 1; i < max_record_hits; i++) {
      const float isect_t = INTEGRATOR_STATE_ARRAY(payload->in_state, shadow_isect, i, t);
      if (isect_t > max_recorded_t) {
        max_recorded_t = isect_t;
        max_recorded_hit = i;
      }
    }

    if (hit.t >= max_recorded_t) {
      return true;
    }

    record_index = max_recorded_hit;
  }

  INTEGRATOR_STATE_ARRAY_WRITE(payload->in_state, shadow_isect, record_index, u) = hit.uv.x;
  INTEGRATOR_STATE_ARRAY_WRITE(payload->in_state, shadow_isect, record_index, v) = hit.uv.y;
  INTEGRATOR_STATE_ARRAY_WRITE(payload->in_state, shadow_isect, record_index, t) = hit.t;
  INTEGRATOR_STATE_ARRAY_WRITE(payload->in_state, shadow_isect, record_index, prim) = hit.primID + prim_offset;
  INTEGRATOR_STATE_ARRAY_WRITE(payload->in_state, shadow_isect, record_index, object) = object_id;
  INTEGRATOR_STATE_ARRAY_WRITE(payload->in_state, shadow_isect, record_index, type) = type;
  return true;
#else
  return false;
#  endif /* __TRANSPARENT_SHADOWS__ */
}

ccl_device_inline bool shadow_intersection_filter_curves(const hiprtRay &ccl_restrict ray,
                                                         ShadowPayload *ccl_restrict payload,
                                                         const hiprtHit &ccl_restrict hit)

{
  const int object_id = kernel_data_fetch(user_instance_id, hit.instanceID);

#  ifdef __SHADOW_LINKING__
  /* It doesn't seem like this is necessary. */
  if (intersection_skip_shadow_link(nullptr, payload->self, object_id)) {
    /* Ignore hit - continue traversal */
    return true;
  }
#  endif

#  ifdef __VISIBILITY_FLAG__
  if ((kernel_data_fetch(objects, object_id).visibility & payload->visibility) == 0) {
    return true;  /* no hit - continue traversal. */
  }
#  endif

  if (intersection_skip_self_shadow(payload->self, object_id, hit.primID)) {
    return true;  /* No hit -continue traversal. */
  }

  if (hit.uv.x == 0.0f || hit.uv.x == 1.0f) {
    /* Continue traversal */
    return true;
  }

#  ifdef __TRANSPARENT_SHADOWS__
  if (payload->num_hits >= payload->max_hits ||
      !(intersection_get_shader_flags(nullptr, hit.primID, payload->prim_type) & SD_HAS_TRANSPARENT_SHADOW))
  {
    return false;
  }

  *payload->r_throughput *= intersection_curve_shadow_transparency(nullptr, object_id, hit.primID, payload->prim_type, hit.uv.x);
  ++payload->num_hits;

  if (*payload->r_throughput < CURVE_SHADOW_TRANSPARENCY_CUTOFF) {
    return false;
  }

  return true;
#  else
  return false;
#  endif /* __TRANSPARENT_SHADOWS__ */
}

ccl_device_inline bool local_intersection_filter(const hiprtRay &ccl_restrict ray,
                                                 LocalPayload *ccl_restrict payload,
                                                 const hiprtHit &ccl_restrict hit)
{
#  ifdef __BVH_LOCAL__
  if (payload->max_hits == 0) {
    return false;  /* Stop search. */
  }

  const int prim_offset = kernel_data_fetch(object_prim_offset, payload->local_object);

  if (intersection_skip_self_local(payload->self, hit.primID + prim_offset)) {
    return true;  /* Continue search. */
  }

  int hit_index = 0;
  if (payload->lcg_state) {
    for (int i = min(payload->max_hits, payload->local_isect->num_hits) - 1; i >= 0; --i) {
      if (hit.t == payload->local_isect->hits[i].t) {
        return true;  /* Continue search. */
      }
    }
    hit_index = payload->local_isect->num_hits++;
    if (payload->local_isect->num_hits > payload->max_hits) {
      hit_index = lcg_step_uint(payload->lcg_state) % payload->local_isect->num_hits;
      if (hit_index >= payload->max_hits) {
        return true;  /* Continue search. */
      }
    }
  }
  else {
    if (payload->local_isect->num_hits && hit.t > payload->local_isect->hits[0].t) {
      return true;
    }
    payload->local_isect->num_hits = 1;
  }

  Intersection *isect = &payload->local_isect->hits[hit_index];
  isect->t = hit.t;
  isect->prim = hit.primID + prim_offset;
  isect->object = payload->local_object;
  isect->type = PRIMITIVE_TRIANGLE;  /* kernel_data_fetch(__objects, object_id).primitive_type; */

  isect->u = hit.uv.x;
  isect->v = hit.uv.y;

  payload->local_isect->Ng[hit_index] = hit.normal;

  return true;
#  else
  return false;
#  endif
}

ccl_device_inline bool volume_intersection_filter(const hiprtRay &ccl_restrict ray,
                                                  RayPayload *ccl_restrict payload,
                                                  const hiprtHit &ccl_restrict hit)
{
  const int object_id = kernel_data_fetch(user_instance_id, hit.instanceID);
  const int prim_offset = kernel_data_fetch(object_prim_offset, object_id);

  if (intersection_skip_self(payload->self, object_id, hit.primID + prim_offset)) {
    return true;
  }

  return (kernel_data_fetch(object_flag, object_id) & SD_OBJECT_HAS_VOLUME) == 0;
}

HIPRT_DEVICE bool intersectFunc(u32 geomType,
                                u32 rayType,
                                const hiprtFuncTableHeader &ccl_restrict tableHeader,
                                const hiprtRay &ccl_restrict ray,
                                void *ccl_restrict payload,
                                hiprtHit &ccl_restrict hit)
{
  switch (tableHeader.numGeomTypes * rayType + geomType) {
    case Curve_Intersect_Function:
    case Curve_Intersect_Shadow:
      return curve_custom_intersect(ray, (RayPayload *)payload, hit);
    case Motion_Triangle_Intersect_Function:
    case Motion_Triangle_Intersect_Shadow:
      return motion_triangle_custom_intersect(ray, (RayPayload *)payload, hit);
    case Motion_Triangle_Intersect_Local:
      return motion_triangle_custom_local_intersect(ray, (LocalPayload *)payload, hit);
    case Motion_Triangle_Intersect_Volume:
      return motion_triangle_custom_volume_intersect(ray, (RayPayload *)payload, hit);
    case Point_Intersect_Function:
    case Point_Intersect_Shadow:
      return point_custom_intersect(ray, (RayPayload *)payload, hit);
    default:
      break;
  }
  return false;
}

HIPRT_DEVICE bool filterFunc(u32 geomType,
                             u32 rayType,
                             const hiprtFuncTableHeader &ccl_restrict tableHeader,
                             const hiprtRay &ccl_restrict ray,
                             void *ccl_restrict payload,
                             const hiprtHit &ccl_restrict hit)
{
  switch (tableHeader.numGeomTypes * rayType + geomType) {
    case Triangle_Filter_Closest:
      return closest_intersection_filter(ray, (RayPayload *)payload, hit);
    case Curve_Filter_Shadow:
      return shadow_intersection_filter_curves(ray, (ShadowPayload *)payload, hit);
    case Triangle_Filter_Shadow:
    case Motion_Triangle_Filter_Shadow:
    case Point_Filter_Shadow:
      return shadow_intersection_filter(ray, (ShadowPayload *)payload, hit);
    case Triangle_Filter_Local:
    case Motion_Triangle_Filter_Local:
      return local_intersection_filter(ray, (LocalPayload *)payload, hit);
    case Triangle_Filter_Volume:
    case Motion_Triangle_Filter_Volume:
      return volume_intersection_filter(ray, (RayPayload *)payload, hit);
    default:
      break;
  }

  return false;
}

#endif
