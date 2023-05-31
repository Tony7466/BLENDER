
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_geom_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ao_lib.glsl)

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  // ivec2 extent = imageSize(in_normal_img);
  ivec2 extent = imageSize(in_normal_img).xy;
  if (any(greaterThanEqual(texel, extent))) {
    return;
  }

  vec2 uv = (vec2(texel) + vec2(0.5)) / vec2(extent);
  vec3 vP, vNg;
  if (!reconstruct_view_position_and_normal_from_depth(hiz_tx, extent, uv, vP, vNg)) {
    /* Do not trace for background */
    imageStore(out_ao_img, ivec3(texel, ao_index), vec4(0.0));
    return;
  }

  vec3 P = transform_point(ViewMatrixInverse, vP);
  vec3 V = cameraVec(P);
  vec3 Ng = transform_direction(ViewMatrixInverse, vNg);
  // vec3 N = imageLoad(in_normal_img, texel).xyz;
  vec3 N = imageLoad(in_normal_img, ivec3(texel, normal_index)).xyz;

  // OcclusionData data = unpack_occlusion_data(texelFetch(horizons_tx, texel, 0));
  OcclusionData data = occlusion_search(vP, hiz_tx, texel, ao_buf.distance, 0.0, 8.0);

  float visibility;
  float visibility_error_out;
  vec3 bent_normal_out;
  occlusion_eval(data, texel, V, N, Ng, 0.0, visibility, visibility_error_out, bent_normal_out);
  /* Scale by user factor */
  visibility = occlusion_pow(saturate(visibility), ao_buf.factor);

  // imageStore(out_ao_img, texel, vec4(visibility));
  imageStore(out_ao_img, ivec3(texel, ao_index), vec4(visibility));
}
