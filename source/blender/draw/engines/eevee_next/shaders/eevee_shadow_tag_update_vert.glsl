/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Virtual shadow-mapping: Update tagging
 *
 * Any updated shadow caster needs to tag the shadow map tiles it was in and is now into.
 * This is done in 2 pass of this same shader. One for past object bounds and one for new object
 * bounds. The bounding boxes are roughly software rasterized (just a plain rectangle) in order to
 * tag the appropriate tiles.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)
#pragma BLENDER_REQUIRE(common_intersect_lib.glsl)
#pragma BLENDER_REQUIRE(common_aabb_lib.glsl)
#pragma BLENDER_REQUIRE(common_debug_shape_lib.glsl)

void main()
{
  tilemap_index = gl_InstanceID % tilemap_count;
  uint resource_id = resource_ids_buf[gl_InstanceID] & 0x7FFFFFFFu;

  ObjectBounds bounds = bounds_buf[resource_id];
  if (!drw_bounds_are_valid(bounds)) {
    gl_Position = vec4(1.0);
    return;
  }

  ShadowTileMapData tilemap = tilemaps_buf[tilemap_index];

  /* Convert from Bone Box shape to 0..1 box. */
  /* TODO(fclem): Remove and use proper coordinates. */
  vec3 lP = max(vec3(0), pos);

  vec3 P = lP.x * bounds.bounding_corners[1].xyz + lP.y * bounds.bounding_corners[2].xyz +
           lP.z * bounds.bounding_corners[3].xyz + bounds.bounding_corners[0].xyz;

  gl_Position = tilemap.winmat * (tilemap.viewmat * vec4(P, 1.0));
}
