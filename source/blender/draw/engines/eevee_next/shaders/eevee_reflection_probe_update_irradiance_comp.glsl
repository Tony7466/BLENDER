
/* Shader to extract spherical harmonics cooefs from octahedral mapped reflection probe. */
/* TODO(jbakker): Use larger dispatch size and atomic add. */

#pragma BLENDER_REQUIRE(eevee_reflection_probe_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)

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

shared SphericalHarmonicL1 cooefs[gl_WorkGroupSize.x];

void main()
{

  /* Initialize workgroup result. */
  if (gl_LocalInvocationID.x == 0) {
    for (int i = 0; i < gl_WorkGroupSize.x; i++) {
      cooefs[i].L0.M0 = vec4(0.0);
      cooefs[i].L1.Mn1 = vec4(0.0);
      cooefs[i].L1.M0 = vec4(0.0);
      cooefs[i].L1.Mp1 = vec4(0.0);
    }
  }

  barrier();

  /* Perform multiple sample. */
  ReflectionProbeData probe_data = reflection_probe_buf[reflection_probe_index];
  uint store_index = gl_LocalInvocationID.x;
  float total_samples = float(gl_WorkGroupSize.x * REFLECTION_PROBE_SH_SAMPLES_PER_GROUP);
  float sample_weight = 4.0 * M_PI / total_samples;
  float sample_offset = float(gl_LocalInvocationID.x * REFLECTION_PROBE_SH_SAMPLES_PER_GROUP);
  for (int sample_index = 0; sample_index < REFLECTION_PROBE_SH_SAMPLES_PER_GROUP; sample_index++)
  {
    vec2 rand = hammersley_2d(sample_index + sample_offset, total_samples);
    vec3 direction = sample_sphere(rand);
    vec4 light = reflection_probes_sample(direction, 0.0, probe_data);
    spherical_harmonics_encode_signal_sample(
        direction, light * sample_weight, cooefs[store_index]);
  }

  barrier();
  if (gl_LocalInvocationID.x != 0) {
    return;
  }

  /* Join results */
  SphericalHarmonicL1 result;
  result.L0.M0 = vec4(0.0);
  result.L1.Mn1 = vec4(0.0);
  result.L1.M0 = vec4(0.0);
  result.L1.Mp1 = vec4(0.0);
  for (int i = 0; i < gl_WorkGroupSize.x; i++) {
    result.L0.M0 += cooefs[i].L0.M0;
    result.L1.Mn1 += cooefs[i].L1.Mn1;
    result.L1.M0 += cooefs[i].L1.M0;
    result.L1.Mp1 += cooefs[i].L1.Mp1;
  }

  ivec2 atlas_coord = ivec2(0, 0);
  atlas_store(result.L0.M0, atlas_coord, 0);
  atlas_store(result.L1.Mn1, atlas_coord, 1);
  atlas_store(result.L1.M0, atlas_coord, 2);
  atlas_store(result.L1.Mp1, atlas_coord, 3);
}