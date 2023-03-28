#pragma BLENDER_REQUIRE(common_view_lib.glsl)

void main()
{
  float dist_sqr = dot(lP, lP);

  /* Discard outside the circle. */
  if (dist_sqr > 1.0) {
    discard;
    return;
  }

  vec3 vN = vec3(lP, sqrt(max(0.0, 1.0 - dist_sqr)));
  vec3 P = mat3(ViewMatrixInverse) * vN;
  out_color = vec4(normal_view_to_world(vN) * 0.5 + 0.5, 1.0);
}
