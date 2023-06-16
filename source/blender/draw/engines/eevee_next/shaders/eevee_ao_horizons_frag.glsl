
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_geom_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ao_lib.glsl)

/**
 * This shader only compute maximum horizon angles for each directions.
 * The final integration is done at the resolve stage with the shading normal.
 */

void main()
{
  vec2 uvs = uvcoordsvar.xy;
  float depth = textureLod(hiz_tx, uvs * hiz_buf.uv_scale, 0.0).r;
  vec3 vP = get_view_space_from_depth(uvs, depth);

  OcclusionData data = ambient_occlusion_disabled_data();
  /* Do not trace for background */
  if (depth != 1.0) {
    ivec2 texel = ivec2(gl_FragCoord.xy);
    data = ambient_occlusion_search(vP, hiz_tx, texel, ao_buf.distance, 0.0, 8.0);
  }

  out_horizons = ambient_occlusion_pack_data(data);
}
