/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * Module that handles light probe update tagging.
 * Lighting data is contained in their respective module `IrradianceCache` and `ReflectionProbes`.
 */

#pragma once

#include "BLI_map.hh"

#include "eevee_sync.hh"

namespace blender::eevee {

class Instance;
class IrradianceCache;

/* -------------------------------------------------------------------- */
/** \name Atlas coord
 * \{ */

struct ReflectionProbeAtlasCoordinate {
  /** On which layer of the texture array is this reflection probe stored. */
  int layer = -1;
  /**
   * Subdivision of the layer. 0 = no subdivision and resolution would be
   * ReflectionProbeModule::MAX_RESOLUTION.
   */
  int layer_subdivision = -1;
  /**
   * Which area of the subdivided layer is the reflection probe located.
   *
   * A layer has (2^layer_subdivision)^2 areas.
   */
  int area_index = -1;

  /* Return the area extent in pixel. */
  int area_extent(int atlas_extent) const
  {
    return atlas_extent >> layer_subdivision;
  }

  /* Coordinate of the area in [0..area_count_per_dimension[ range. */
  int2 area_location() const
  {
    const int area_count_per_dimension = 1 << layer_subdivision;
    return int2(area_index % area_count_per_dimension, area_index / area_count_per_dimension);
  }

  /* Coordinate of the bottom left corner of the area in [0..atlas_extent[ range. */
  int2 area_offset(int atlas_extent) const
  {
    return area_location() * area_extent(atlas_extent);
  }

  ReflectionProbeCoordinate as_sampling_coord(int atlas_extent) const
  {
    /**
     * We want to cover the last mip exactly at the pixel center to reduce padding texels and
     * interpolation artifacts.
     * This is a diagram of a 2px^2 map with `c` being the texels corners and `x` the pixels
     * centers.
     *
     * c-------c-------c
     * |       |       |
     * |   x   |   x   | <
     * |       |       |  |
     * c-------c-------c  | sampling area
     * |       |       |  |
     * |   x   |   x   | <
     * |       |       |
     * c-------c-------c
     *     ^-------^
     *       sampling area
     */
    /* First level only need half a pixel of padding around the sampling area. */
    const int mip_max_lvl_padding = 1;
    const int mip_min_lvl_padding = mip_max_lvl_padding << REFLECTION_PROBE_MIPMAP_LEVELS;
    /* Extent and offset in mip 0 texels. */
    const int sampling_area_extent = area_extent(atlas_extent) - mip_min_lvl_padding;
    const int2 sampling_area_offset = area_offset(atlas_extent) + mip_min_lvl_padding / 2;
    /* Convert to atlas UVs. */
    ReflectionProbeCoordinate coord;
    coord.scale = sampling_area_extent / float(atlas_extent);
    coord.offset = float2(sampling_area_offset) / float(atlas_extent);
    coord.layer = layer;
    return coord;
  }

  ReflectionProbeWriteCoordinate as_write_coord(int atlas_extent, int mip_lvl) const
  {
    ReflectionProbeWriteCoordinate coord;
    coord.extent = atlas_extent >> (layer_subdivision + mip_lvl);
    coord.offset = (area_location() * coord.extent) >> mip_lvl;
    coord.layer = layer;
    return coord;
  }
};

/** \} */

struct LightProbe {
  bool used = false;
  bool initialized = false;
  /* NOTE: Might be not needed if depsgraph updates work as intended. */
  bool updated = false;
};

struct IrradianceGrid : public LightProbe, IrradianceGridData {
  /** Copy of the transform matrix. */
  float4x4 object_to_world;
  /** Precomputed inverse transform with normalized axes. No position. Used for rotating SH. */
  float4x4 world_to_object;
  /**
   * Reference to the light-cache data.
   * Do not try to dereference it before LightProbeModule::end_sync() as the grid could
   * already have been freed (along with its cache). It is only safe to dereference after the
   * pruning have been done.
   */
  const LightProbeObjectCache *cache = nullptr;
  /** List of associated atlas bricks that are used by this grid. */
  Vector<IrradianceBrickPacked> bricks;
  /** True if the grid needs to be reuploaded & re-composited with other light-grids. */
  bool do_update;
  /** Index of the grid inside the grid UBO. */
  int grid_index;
  /** Copy of surfel density for debugging purpose. */
  float surfel_density;
  /** Copy of DNA members. */
  float validity_threshold;
  float dilation_threshold;
  float dilation_radius;
  float intensity;
  /** Display irradiance samples in the viewport. */
  bool viewport_display;
  float viewport_display_size;
};

struct ReflectionCube : public LightProbe, ReflectionProbeData {
  /** Used to sort the probes by priority. */
  float volume;
  /** True if the area in the atlas needs to be updated. */
  bool do_render = true;
  /** False if the area in the atlas contains undefined data. */
  bool use_for_render = false;
  /** Far and near clipping distances for rendering. */
  float2 clipping_distances;
  /** Atlas region this probe is rendered at (or will be rendered at). */
  ReflectionProbeAtlasCoordinate atlas_coord;

  /** Display debug spheres in the viewport. */
  bool viewport_display;
  float viewport_display_size;

  /* Return the subdivision level for the requested probe resolution.
   * Result is safely clamped to max resolution. */
  static int subdivision_level_get(const int max_resolution,
                                   const eLightProbeResolution probe_resolution)
  {
    return max_ii(int(log2(max_resolution)) - int(probe_resolution), 0);
  }
};

struct ProbePlane : public LightProbe, ProbePlanarData {
  /* Copy of object matrices. */
  float4x4 plane_to_world;
  float4x4 world_to_plane;
  /* Offset to the clipping plane in the normal direction. */
  float clipping_offset;
  /* Index in the resource array. */
  int resource_index;
  /** Display a debug plane in the viewport. */
  bool viewport_display = false;

 public:
  /**
   * Update the ProbePlanarData part of the struct.
   * `view` is the view we want to render this probe with.
   */
  void set_view(const draw::View &view, int layer_id);

  /**
   * Create the reflection clip plane equation that clips along the XY plane of the given
   * transform. The `clip_offset` will push the clip plane a bit further to avoid missing pixels in
   * reflections. The transform does not need to be normalized but is expected to be orthogonal.
   * \note Only works after `set_view` was called.
   */
  float4 reflection_clip_plane_get()
  {
    return float4(-normal, math::dot(normal, plane_to_world.location()) - clipping_offset);
  }

 private:
  /**
   * Create the reflection matrix that reflect along the XY plane of the given transform.
   * The transform does not need to be normalized but is expected to be orthogonal.
   */
  float4x4 reflection_matrix_get()
  {
    return plane_to_world * math::from_scale<float4x4>(float3(1, 1, -1)) * world_to_plane;
  }
};

class LightProbeModule {
  friend class IrradianceCache;
  friend class PlanarProbeModule;
  friend class ReflectionProbeModule;

 private:
  Instance &inst_;

  /** Light Probe map to detect deletion and store associated data. */
  Map<ObjectKey, IrradianceGrid> grid_map_;
  Map<ObjectKey, ReflectionCube> cube_map_;
  Map<ObjectKey, ProbePlane> plane_map_;
  /** True if a light-probe update was detected. */
  bool grid_update_;
  bool cube_update_;
  bool plane_update_;
  /** True if the auto bake feature is enabled & available in this context. */
  bool auto_bake_enabled_;

 public:
  LightProbeModule(Instance &inst) : inst_(inst){};
  ~LightProbeModule(){};

  void begin_sync();

  void sync_probe(const Object *ob, ObjectHandle &handle);

  void end_sync();

 private:
  void sync_cube(const Object *ob, ObjectHandle &handle);
  void sync_grid(const Object *ob, ObjectHandle &handle);
  void sync_plane(const Object *ob, ObjectHandle &handle);
};

}  // namespace blender::eevee
