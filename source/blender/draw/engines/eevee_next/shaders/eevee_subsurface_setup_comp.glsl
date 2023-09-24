/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Process in screen space the diffuse radiance input to mimic subsurface transmission.
 *
 * This implementation follows the technique described in the siggraph presentation:
 * "Efficient screen space subsurface scattering Siggraph 2018"
 * by Evgenii Golubev
 *
 * But, instead of having all the precomputed weights for all three color primaries,
 * we precompute a weight profile texture to be able to support per pixel AND per channel radius.
 */

#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)

void main(void)
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);

  GBufferData gbuf = gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_color_tx, texel);

  if (gbuf.has_diffuse && gbuf.diffuse.sss_id != 0u) {
    vec3 radiance = imageLoad(direct_light_img, texel).rgb +
                    imageLoad(indirect_light_img, texel).rgb;

    float sss_id_packed = gbuffer_object_id_f16_pack(gbuf.diffuse.sss_id);

    imageStore(out_radiance_id_img, texel, vec4(radiance, sss_id_packed));
  }
  else {
    imageStore(out_radiance_id_img, texel, vec4(0.0));
  }
}
