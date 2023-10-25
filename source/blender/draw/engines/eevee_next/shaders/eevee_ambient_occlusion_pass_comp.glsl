/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(draw_view_reconstruction_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ambient_occlusion_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_horizon_scan_eval_lib.glsl)

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  ivec2 extent = imageSize(in_normal_img).xy;
  if (any(greaterThanEqual(texel, extent))) {
    return;
  }

  SurfaceReconstructResult surf = view_reconstruct_from_depth(hiz_tx, extent, texel);
  if (surf.is_background) {
    /* Do not trace for background */
    imageStore(out_ao_img, ivec3(texel, out_ao_img_layer_index), vec4(0.0));
    return;
  }

  vec3 P = drw_point_view_to_world(surf.vP);
  vec3 V = drw_world_incident_vector(P);
  vec3 N = imageLoad(in_normal_img, ivec3(texel, in_normal_img_layer_index)).xyz;
  vec3 vN = drw_normal_world_to_view(N);

  vec2 noise = interlieved_gradient_noise(
      vec2(texel), vec2(3, 5), sampling_rng_2D_get(SAMPLING_AO_U));

  vec3 ambient_occlusion = horizon_scan_eval(surf.vP,
                                             vN,
                                             hiz_tx,
                                             noise,
                                             uniform_buf.ao.pixel_size,
                                             uniform_buf.ao.distance,
                                             uniform_buf.ao.thickness,
                                             false,
                                             8);

  /* We can have some float imprecision because of the weighted accumulation. */
  if (ambient_occlusion.r >= 0.98) {
    ambient_occlusion = vec3(1.0);
  }

  imageStore(out_ao_img, ivec3(texel, out_ao_img_layer_index), saturate(ambient_occlusion.rrrr));
}
