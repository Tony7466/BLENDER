
/**
 * Load an input lightgrid cache texture into the atlas.
 *
 * Each thread group will load a brick worth of data and add the needed padding texels.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_lightprobe_eval_lib.glsl)

void atlas_store(vec4 sh_coefficient, ivec2 atlas_coord, int layer)
{
  imageStore(irradiance_atlas_img,
             ivec3(atlas_coord, layer * IRRADIANCE_GRID_BRICK_SIZE) + ivec3(gl_LocalInvocationID),
             sh_coefficient);
}

void main()
{
  int brick_index = lightprobe_irradiance_grid_brick_index_get(grids_infos_buf[grid_index],
                                                               ivec3(gl_WorkGroupID));

  /* Brick coordinate in the source grid. */
  ivec3 brick_coord = ivec3(gl_WorkGroupID);
  /* Add padding border to allow bilinear filtering. */
  ivec3 texel_coord = brick_coord * (IRRADIANCE_GRID_BRICK_SIZE - 1) + ivec3(gl_LocalInvocationID);
  ivec3 input_coord = min(texel_coord, textureSize(irradiance_a_tx, 0) - 1);

  /* Brick coordinate in the destination atlas. */
  IrradianceBrick brick = irradiance_brick_unpack(bricks_infos_buf[brick_index]);
  ivec2 output_coord = ivec2(brick.atlas_coord);

  SphericalHarmonicL1 sh_local;
  sh_local.L0.M0 = texelFetch(irradiance_a_tx, input_coord, 0);
  sh_local.L1.Mn1 = texelFetch(irradiance_b_tx, input_coord, 0);
  sh_local.L1.M0 = texelFetch(irradiance_c_tx, input_coord, 0);
  sh_local.L1.Mp1 = texelFetch(irradiance_d_tx, input_coord, 0);

  /* Load visibility separately as it might not be in the same texture. */
  sh_local.L0.M0.a = texelFetch(visibility_a_tx, input_coord, 0).a;
  sh_local.L1.Mn1.a = texelFetch(visibility_b_tx, input_coord, 0).a;
  sh_local.L1.M0.a = texelFetch(visibility_c_tx, input_coord, 0).a;
  sh_local.L1.Mp1.a = texelFetch(visibility_d_tx, input_coord, 0).a;

  /* Rotate Spherical Harmonic into world space. */
  mat3 grid_to_world_rot = normalize(mat3(grids_infos_buf[grid_index].world_to_grid_transposed));
  sh_local = spherical_harmonics_rotate(grid_to_world_rot, sh_local);

  SphericalHarmonicL1 sh_visibility;
  sh_visibility.L0.M0 = sh_local.L0.M0.aaaa;
  sh_visibility.L1.Mn1 = sh_local.L1.Mn1.aaaa;
  sh_visibility.L1.M0 = sh_local.L1.M0.aaaa;
  sh_visibility.L1.Mp1 = sh_local.L1.Mp1.aaaa;

  vec3 P = lightprobe_irradiance_grid_sample_position(
      grid_local_to_world, grids_infos_buf[grid_index].grid_size, input_coord);

  SphericalHarmonicL1 sh_distant = lightprobe_irradiance_sample(P);
  /* Mask distant lighting by local visibility. */
  sh_distant = spherical_harmonics_triple_product(sh_visibility, sh_distant);
  /* Add local lighting to distant lighting. */
  sh_local = spherical_harmonics_add(sh_local, sh_distant);

  atlas_store(sh_local.L0.M0, output_coord, 0);
  atlas_store(sh_local.L1.Mn1, output_coord, 1);
  atlas_store(sh_local.L1.M0, output_coord, 2);
  atlas_store(sh_local.L1.Mp1, output_coord, 3);
}
