#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_geom_lib.glsl)

void main()
{
  surfel_index = gl_InstanceID;
  Surfel surfel = surfels_buf[surfel_index];

  const vec3 verts[4] = vec3[4](vec3(-1, 1, 0), vec3(-1, -1, 0), vec3(1, 1, 0), vec3(1, -1, 0));
  vec3 lP = verts[gl_VertexID];

  vec3 N = surfel.normal.xyz;
  vec3 T, B;
  make_orthonormal_basis(N, T, B);

  mat4 model_matrix = mat4(vec4(T * surfel_radius, 0),
                           vec4(B * surfel_radius, 0),
                           vec4(N * surfel_radius, 0),
                           vec4(surfel.position.xyz, 1));

  P = (model_matrix * vec4(lP, 1)).xyz;

  gl_Position = point_world_to_ndc(P);
}
