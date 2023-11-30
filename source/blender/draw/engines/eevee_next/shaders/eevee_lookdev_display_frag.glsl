void main()
{
  vec4 color = texture(color_tx, uv_coord, 0);

  color.a = 1.0 - color.a;
  out_color = color;
}