#pragma BLENDER_REQUIRE(eevee_volume_lib.glsl)

/* Needed includes for shader nodes. */
#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_attribute_lib.glsl)
#pragma BLENDER_REQUIRE(closure_type_lib.glsl)
#pragma BLENDER_REQUIRE(closure_eval_volume_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_light_eval_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_attributes_lib.glsl)

/* Based on Frosbite Unified Volumetric.
 * https://www.ea.com/frostbite/news/physically-based-unified-volumetric-rendering-in-frostbite */

/* Store volumetric properties into the froxel textures. */

GlobalData init_globals(void)
{
  GlobalData surf;
  surf.P = worldPosition;
  surf.N = vec3(0.0);
  surf.Ng = vec3(0.0);
  surf.is_strand = false;
  surf.hair_time = 0.0;
  surf.hair_thickness = 0.0;
  surf.hair_strand_id = 0;
  surf.barycentric_coords = vec2(0.0);
  surf.barycentric_dists = vec3(0.0);
  surf.ray_type = RAY_TYPE_CAMERA;
  surf.ray_depth = 0.0;
  surf.ray_length = distance(surf.P, cameraPos);
  return surf;
}

Closure nodetree_volume();
void attrib_load();

void main()
{
  ivec3 volume_cell = ivec3(ivec2(gl_FragCoord.xy), volume_geom_iface.slice);
  vec3 ndc_cell = volume_to_ndc((vec3(volume_cell) + volumes_buf.jitter) *
                                volumes_buf.inv_tex_size);

  viewPosition = get_view_space_from_depth(ndc_cell.xy, ndc_cell.z);
  worldPosition = point_view_to_world(viewPosition);
#ifdef MESH_SHADER
  objectPosition = point_world_to_object(worldPosition);
  g_orco = OrcoTexCoFactors[0].xyz + objectPosition * OrcoTexCoFactors[1].xyz;

  if (any(lessThan(g_orco, vec3(0.0))) || any(greaterThan(g_orco, vec3(1.0)))) {
    /* NOTE: Discard is not an explicit return in Metal prior to versions 2.3.
     * adding return after discard ensures consistent behavior and avoids GPU
     * side-effects where control flow continues with undefined values. */
    discard;
    return;
  }
#else /* WORLD_SHADER */
  g_orco = worldPosition;
#endif

  g_data = init_globals();
#ifndef NO_ATTRIB_LOAD
  attrib_load();
#endif
  Closure cl = nodetree_volume();
#ifdef MESH_SHADER
  cl.scatter *= drw_volume.density_scale;
  cl.absorption *= drw_volume.density_scale;
  cl.emission *= drw_volume.density_scale;
#endif

  volumeScattering = vec4(cl.scatter, 1.0);
  volumeExtinction = vec4(cl.absorption + cl.scatter, 1.0);
  volumeEmissive = vec4(cl.emission, 1.0);
  /* Do not add phase weight if no scattering. */
  if (all(equal(cl.scatter, vec3(0.0)))) {
    volumePhase = vec4(0.0);
  }
  else {
    volumePhase = vec4(cl.anisotropy, vec3(1.0));
  }
}
