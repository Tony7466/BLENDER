
#pragma BLENDER_REQUIRE(eevee_volume_lib.glsl)

/* Based on Frosbite Unified Volumetric.
 * https://www.ea.com/frostbite/news/physically-based-unified-volumetric-rendering-in-frostbite */

/* Step 2 : Evaluate all light scattering for each froxels.
 * Also do the temporal reprojection to fight aliasing artifacts. */

#ifdef VOLUME_LIGHTING

vec3 volume_scatter_light_eval(vec3 P, vec3 V, uint l_idx, float s_anisotropy)
{
  LightData ld = light_buf[l_idx];

  if (ld.volume_power == 0.0) {
    return vec3(0);
  }

  vec4 l_vector;
  l_vector.xyz = light_volume_light_vector(ld, P);
  l_vector.w = length(l_vector.xyz);

  vec3 L;
  float dist;
  light_vector_get(ld, P, L, dist);
  float attenuation = light_attenuation(ld, L, dist);

#  if 0
  /* TODO(Miguel Pozo): Was shadowing applied twice ? */
  float vis = light_visibility(ld, P, l_vector);
#  else
  float vis = attenuation;
#  endif

  if (vis < 1e-4) {
    return vec3(0);
  }

  vec3 Li = light_volume(ld, l_vector) * light_volume_shadow(ld, P, l_vector, volumeExtinction);

  return Li * vis * phase_function(-V, l_vector.xyz / l_vector.w, s_anisotropy);
}

#endif

void main()
{
  ivec3 volume_cell = ivec3(ivec2(gl_FragCoord.xy), volume_geom_iface.slice);

  /* Emission */
  outScattering = texelFetch(volumeEmission, volume_cell, 0);
  outTransmittance = texelFetch(volumeExtinction, volume_cell, 0);
  vec3 s_scattering = texelFetch(volumeScattering, volume_cell, 0).rgb;
  vec3 volume_ndc = volume_to_ndc((vec3(volume_cell) + volumes_buf.jitter) *
                                  volumes_buf.inv_tex_size);
  vec3 vP = get_view_space_from_depth(volume_ndc.xy, volume_ndc.z);
  vec3 P = point_view_to_world(vP);
  vec3 V = cameraVec(P);

  vec2 phase = texelFetch(volumePhase, volume_cell, 0).rg;
  float s_anisotropy = phase.x / max(1.0, phase.y);

  /* Environment : Average color. */
  outScattering.rgb += irradiance_volumetric(P) * s_scattering * phase_function_isotropic();

#ifdef VOLUME_LIGHTING

  LIGHT_FOREACH_BEGIN_DIRECTIONAL (light_cull_buf, l_idx) {
    outScattering.rgb += volume_scatter_light_eval(P, V, l_idx, s_anisotropy) * s_scattering;
  }
  LIGHT_FOREACH_END

  LIGHT_FOREACH_BEGIN_LOCAL (
      light_cull_buf, light_zbin_buf, light_tile_buf, gl_FragCoord.xy, vP.z, l_idx) {
    outScattering.rgb += volume_scatter_light_eval(P, V, l_idx, s_anisotropy) * s_scattering;
  }
  LIGHT_FOREACH_END

#endif

  /* Temporal supersampling */
  /* Note : this uses the cell non-jittered position (texel center). */
  vec3 curr_ndc = volume_to_ndc(vec3(gl_FragCoord.xy, float(volume_geom_iface.slice) + 0.5) *
                                volumes_buf.inv_tex_size);
  vec3 wpos = get_world_space_from_depth(curr_ndc.xy, curr_ndc.z);
  vec3 prev_ndc = project_point(volumes_buf.prev_view_projection_matrix, wpos);
  vec3 prev_volume = ndc_to_volume(prev_ndc * 0.5 + 0.5);

  if ((volumes_buf.history_alpha > 0.0) && all(greaterThan(prev_volume, vec3(0.0))) &&
      all(lessThan(prev_volume, vec3(1.0)))) {
    vec4 h_Scattering = texture(historyScattering, prev_volume);
    vec4 h_Transmittance = texture(historyTransmittance, prev_volume);
    outScattering = mix(outScattering, h_Scattering, volumes_buf.history_alpha);
    outTransmittance = mix(outTransmittance, h_Transmittance, volumes_buf.history_alpha);
  }

  /* Catch NaNs */
  if (any(isnan(outScattering)) || any(isnan(outTransmittance))) {
    outScattering = vec4(0.0);
    outTransmittance = vec4(1.0);
  }
}
