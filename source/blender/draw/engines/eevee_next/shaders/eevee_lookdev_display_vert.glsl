void main()
{
  uint vert_index = gl_VertexID < 3 ? gl_VertexID : gl_VertexID - 2;

  vec2 uv = vec2(vert_index / 2, vert_index % 2);
  uv_coord = vec3(uv, gpu_InstanceIndex);

  vec2 scale = textureSize(color_tx, 0).xy * invertedViewportSize;
  vec2 offset = vec2(0.8, -0.9) - vec2(1.5, 0.0) * scale * gpu_InstanceIndex;
  vec2 co = uv * scale + offset;
  gl_Position = vec4(co, 0.0, 1.0);
}