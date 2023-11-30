void main()
{
  vec2 texture_size = textureSize(color_tx, 0).xy;
  vec2 texel_size = vec2(1.0) / texture_size;

  float distance_from_center = distance(uv_coord.xy, vec2(0.5));
  if (distance_from_center > 0.5) {
    discard;
    return;
  }
  float smooth_size = texel_size.x * 1.5;
  float smooth_offset = texel_size.x * 0.5;
  float factor = (distance_from_center - 0.5 + smooth_offset + smooth_size) * (1.0 / smooth_size);
  float alpha = clamp(1.0 - factor, 0.0, 1.0);

  vec4 color = vec4(texture(color_tx, uv_coord, 0).rgb, alpha);
  out_color = color;

  /* Ensure balls are on top of overlays. */
  gl_FragDepth = 0.0;
}