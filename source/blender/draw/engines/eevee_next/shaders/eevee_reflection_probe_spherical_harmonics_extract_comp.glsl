
/* Shader to extract spherical harmonics cooefs from octahedral mapped reflection probe. */
/* NOTE: this is a stupid implementation as it uses a single thread. Eventually this should
 * be done using a 16*16 group size.
 */

#pragma BLENDER_REQUIRE(eevee_reflection_probe_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)

void main()
{
  ReflectionProbeData probe_data = reflection_probe_buf[reflection_probe_index];

  SphericalHarmonicL1 cooefs;
  cooefs.L0.M0 = vec4(0.0);
  cooefs.L1.Mn1 = vec4(0.0);
  cooefs.L1.M0 = vec4(0.0);
  cooefs.L1.Mp1 = vec4(0.0);

  ivec3 texture_size = textureSize(reflectionProbes, 0);
  int resolution = (texture_size.x >> probe_data.layer_subdivision) -
                   int(2 * REFLECTION_PROBE_BORDER_SIZE);
  float sample_weight = 1.0 / float(resolution * resolution);
  vec2 texel_size = vec2(1.0 / resolution);

  for (int x = 0; x < resolution; x++) {
    for (int y = 0; y < resolution; y++) {
      vec2 packed_uv = (vec2(REFLECTION_PROBE_BORDER_SIZE) + vec2(float(x), float(y))) *
                       texel_size;
      vec2 octahedral_uv = octahedral_uv_from_layer_texture_coords(
          packed_uv, probe_data, texel_size);
      vec3 direction = octahedral_uv_to_direction(octahedral_uv);
      vec4 light = reflection_probes_sample(direction, 0.0, probe_data);
      spherical_harmonics_encode_signal_sample(direction, light * sample_weight, cooefs);
    }
  }

  // TODO(jbakker): Store result
}