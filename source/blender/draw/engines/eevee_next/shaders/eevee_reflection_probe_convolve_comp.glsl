/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Shader to convert cube-map to octahedral projection. */

#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_reflection_probe_mapping_lib.glsl)

float sample_weight(vec3 out_direction, vec3 in_direction)
{
  out_direction = normalize(out_direction);
  in_direction = normalize(in_direction);

  float cos_theta = clamp(dot(out_direction, in_direction), 0.0, 1.0);
  float m = square(mip_roughness);
  /* Map GGX roughness to spherical gaussian sharpness.
   * From "SG Series Part 4: Specular Lighting From an SG Light Source" by MJP
   * https://therealmjp.github.io/posts/sg-series-part-4-specular-lighting-from-an-sg-light-source/
   */
  vec3 N = out_direction;
  vec3 H = normalize(out_direction + in_direction);
  float NH = clamp(dot(N, H), 0.0, 1.0);
  /* GGX. */
  // return exp(-square(acos(NH) / m));
  /* Spherical Gaussian. */
  return exp(2.0 * (NH - 1.0) / square(m));
}

void convolve_sample(ivec2 in_local_texel,
                     vec3 out_direction,
                     SphereProbeUvArea sample_coord,
                     SphereProbePixelArea in_texel_area,
                     inout vec4 radiance_accum,
                     inout float weight_accum)
{

  in_local_texel = in_local_texel % in_texel_area.extent;

  int in_atlas_mip_size = imageSize(in_atlas_mip_img).x;
  vec3 in_direction = sphere_probe_texel_to_direction(
      in_local_texel, in_texel_area, sample_coord, in_atlas_mip_size);

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

  int out_atlas_mip_size = imageSize(in_atlas_mip_img).x / 2;
  vec3 out_direction = sphere_probe_texel_to_direction(
      out_local_texel, out_texel_area, sample_coord, out_atlas_mip_size);

  ivec2 out_texel = out_texel_area.offset + out_local_texel;
  /* Assume we always input the previous mipmap. */
  const int mip_scaling = 2;

  SphereProbePixelArea in_texel_area = out_texel_area;
  in_texel_area.offset *= mip_scaling;
  in_texel_area.extent *= mip_scaling;

  float weight = 0.0;
  vec4 radiance = vec4(0.0);
  /* Bottom left corner of the area processed for the output texel. */
  ivec2 in_texel_base = out_texel * mip_scaling;
  /* TODO(fclem): Could derive the radius to process by taking the angle which encompass most of
   * the gaussian (using an epsilon threshold) and then derive a number of mip pixels from it. */
  int process_area_radius = 40 * (mip_scaling / 2);
  for (int y = -(process_area_radius - 1); y <= process_area_radius; y += mip_scaling / 2) {
    for (int x = -(process_area_radius - 1); x <= process_area_radius; x += mip_scaling / 2) {
      convolve_sample(in_texel_base + ivec2(x, y),
                      out_direction,
                      sample_coord,
                      in_texel_area,
                      radiance,
                      weight);
    }
  }
  radiance *= safe_rcp(weight);

  imageStore(out_atlas_mip_img, ivec3(out_texel, out_texel_area.layer), radiance);
}
