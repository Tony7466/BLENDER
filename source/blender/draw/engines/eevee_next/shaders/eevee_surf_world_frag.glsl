
/**
 * Background used to shade the world.
 *
 * Outputs shading parameter per pixel using a set of randomized BSDFs.
 **/

#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_attributes_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_surf_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_nodetree_lib.glsl)

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);

  /* Clear AOVs first. In case the material renders to them. */
  for (int i = 0; i < AOV_MAX && i < rp_buf.aovs.color_len; i++) {
    imageStore(rp_color_img, ivec3(texel, rp_buf.color_len + i), vec4(0));
  }
  for (int i = 0; i < AOV_MAX && i < rp_buf.aovs.value_len; i++) {
    imageStore(rp_value_img, ivec3(texel, rp_buf.value_len + i), vec4(0));
  }

  init_globals();
  /* View position is passed to keep accuracy. */
  g_data.N = normal_view_to_world(viewCameraVec(interp.P));
  g_data.Ng = g_data.N;
  g_data.P = -g_data.N + cameraPos;
  attrib_load();

  nodetree_surface();

  g_holdout = saturate(g_holdout);

  out_background.rgb = safe_color(g_emission) * (1.0 - g_holdout);
  out_background.a = saturate(avg(g_transmittance)) * g_holdout;

  /* World opacity. */
  out_background = mix(vec4(0.0, 0.0, 0.0, 1.0), out_background, world_opacity_fade);

  /* Clear Render Buffers. */
  vec4 environment = out_background;
  environment.a = 1.0 - environment.a;
  environment.rgb *= environment.a;
  RP_OUTPUT_COLOR(environment_id, texel, environment);
  vec4 clear_color = vec4(0.0, 0.0, 0.0, 1.0);
  RP_OUTPUT_COLOR(normal_id, texel, clear_color);
  RP_OUTPUT_COLOR(diffuse_light_id, texel, clear_color);
  RP_OUTPUT_COLOR(specular_light_id, texel, clear_color);
  RP_OUTPUT_COLOR(diffuse_color_id, texel, clear_color);
  RP_OUTPUT_COLOR(specular_color_id, texel, clear_color);
  RP_OUTPUT_COLOR(emission_id, texel, clear_color);
  imageStore(rp_cryptomatte_img, texel, vec4(0.0));
}
