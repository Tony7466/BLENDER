
/* Shader to extract spherical harmonics cooefs from octahedral mapped reflection probe. */
/* NOTE: this is a stupid implementation as it uses a single thread. Eventually this should
 * be done using a 16*16 group size.
 */

#pragma BLENDER_REQUIRE(eevee_reflection_probe_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)

void atlas_store(vec4 sh_coefficient, ivec2 atlas_coord, int layer)
{
  for (int x = 0; x < IRRADIANCE_GRID_BRICK_SIZE; x++) {
    for (int y = 0; y < IRRADIANCE_GRID_BRICK_SIZE; y++) {
      for (int z = 0; z < IRRADIANCE_GRID_BRICK_SIZE; z++) {
        ivec3 brick_coord = ivec3(x, y, z);
        imageStore(irradiance_atlas_img,
                   ivec3(atlas_coord, layer * IRRADIANCE_GRID_BRICK_SIZE) + brick_coord,
                   sh_coefficient);
      }
    }
  }
}

void main()
{
  ReflectionProbeData probe_data = reflection_probe_buf[reflection_probe_index];

  SphericalHarmonicL1 cooefs;
  cooefs.L0.M0 = vec4(0.0);
  cooefs.L1.Mn1 = vec4(0.0);
  cooefs.L1.M0 = vec4(0.0);
  cooefs.L1.Mp1 = vec4(0.0);

  ivec3 texture_size = textureSize(reflectionProbes, 0);
  int max_subdivisions = int(log2(texture_size.x));
  /* Find out at what subdivision level the probe has its 64x64 probe texture stored.
   * Currently all probe resolutions have somewhere stored this resolution. */
  const int subdivision_64 = 6;
  int layer_i = clamp(
      subdivision_64 - probe_data.layer_subdivision, 0, REFLECTION_PROBE_MIPMAP_LEVELS - 1);
  float layer_fl = float(layer_i);

  int base_resolution = (texture_size.x >> layer_i);
  int resolution = base_resolution - int(2 * REFLECTION_PROBE_BORDER_SIZE);
  float sample_weight = 4.0 * M_PI / float(resolution * resolution);
  vec2 texel_size = vec2(1.0 / base_resolution);

  for (int x = 0; x < resolution; x++) {
    for (int y = 0; y < resolution; y++) {
      vec2 packed_uv = (vec2(REFLECTION_PROBE_BORDER_SIZE) + vec2(x, y)) * texel_size;
      vec2 octahedral_uv = octahedral_uv_from_layer_texture_coords(
          packed_uv, probe_data, texel_size);
      vec3 direction = octahedral_uv_to_direction(octahedral_uv);
      vec4 light = reflection_probes_sample(direction, layer_fl, probe_data);
      light.xyz = min(light.xyz, vec3(REFLECTION_PROBE_MAX_LIGHT));
      spherical_harmonics_encode_signal_sample(direction, light * sample_weight, cooefs);
    }
  }

  ivec2 atlas_coord = ivec2(0, 0);
  atlas_store(cooefs.L0.M0, atlas_coord, 0);
  atlas_store(cooefs.L1.Mn1, atlas_coord, 1);
  atlas_store(cooefs.L1.M0, atlas_coord, 2);
  atlas_store(cooefs.L1.Mp1, atlas_coord, 3);
}