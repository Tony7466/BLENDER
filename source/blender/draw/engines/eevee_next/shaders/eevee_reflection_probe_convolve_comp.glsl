/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Shader to convert cube-map to octahedral projection. */

#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_reflection_probe_mapping_lib.glsl)

float sample_weight(vec3 out_direction, vec3 in_direction)
{
  return 1.0;
}

void convolve_sample(ivec2 in_local_texel,
                     vec3 out_direction,
                     inout vec4 radiance_accum,
                     inout float weight_accum)
{
  SphereProbeUvArea sample_coord = reinterpret_as_atlas_coord(probe_coord_packed);
  SphereProbePixelArea in_texel_area = reinterpret_as_write_coord(write_coord_packed);
  in_texel_area.offset *= 2;
  in_texel_area.extent *= 2;

  int atlas_mip_size = imageSize(out_atlas_mip_img).x;
  vec3 in_direction = sphere_probe_texel_to_direction(
      in_local_texel, in_texel_area, sample_coord, atlas_mip_size);

  float weight = sample_weight(out_direction, in_direction);
  vec4 radiance = imageLoad(in_atlas_mip_img, ivec3(in_local_texel, in_texel_area.layer));

  radiance_accum += radiance * weight;
  weight_accum += weight;
}

void main()
{
  SphereProbeUvArea sample_coord = reinterpret_as_atlas_coord(probe_coord_packed);
  SphereProbePixelArea out_texel_area = reinterpret_as_write_coord(write_coord_packed);

  /* Texel in probe. */
  ivec2 out_local_texel = ivec2(gl_GlobalInvocationID.xy);

  /* Exit when pixel being written doesn't fit in the area reserved for the probe. */
  if (any(greaterThanEqual(out_local_texel, ivec2(out_texel_area.extent)))) {
    return;
  }

  int output_atlas_mip_size = imageSize(in_atlas_mip_img).x / 2;
  vec3 out_direction = sphere_probe_texel_to_direction(
      out_local_texel, out_texel_area, sample_coord, output_atlas_mip_size);

  ivec2 out_texel = out_texel_area.offset + out_local_texel;

  float weight = 0.0;
  vec4 radiance = vec4(0.0);
  /* Bottom left corner of the area processed for the output texel. */
  ivec2 in_texel_base = out_texel * 2;
  const int process_area_size = 2;
  const int process_area_start = 0;
  for (int y = process_area_start; y < process_area_size; y++) {
    for (int x = process_area_start; x < process_area_size; x++) {
      convolve_sample(in_texel_base + ivec2(x, y), out_direction, radiance, weight);
    }
  }
  radiance *= safe_rcp(weight);

  imageStore(out_atlas_mip_img, ivec3(out_texel, out_texel_area.layer), radiance);
}
