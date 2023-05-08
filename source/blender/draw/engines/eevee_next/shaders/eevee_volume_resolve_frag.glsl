
#pragma BLENDER_REQUIRE(eevee_volume_lib.glsl)

/* Based on Frosbite Unified Volumetric.
 * https://www.ea.com/frostbite/news/physically-based-unified-volumetric-rendering-in-frostbite */

/* Step 4 : Apply final integration on top of the scene color. */

void main()
{
  vec2 uvs = gl_FragCoord.xy / vec2(textureSize(in_scene_depth, 0));
  float scene_depth = texture(in_scene_depth, uvs).r;

  vec3 transmittance, scattering;
  volumetric_resolve(uvs, scene_depth, transmittance, scattering);

  out_radiance = vec4(scattering, 0.0);
  out_transmittance = vec4(transmittance, saturate(avg(transmittance)));
}
