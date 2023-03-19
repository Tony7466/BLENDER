
void main()
{
  Surfel surfel = surfels_buf[surfel_index];

  switch (eDebugMode(debug_mode)) {
    default:
    case DEBUG_IRRADIANCE_CACHE_SURFELS_NORMAL:
      out_color = vec4(surfel.normal, 1.0);
      break;
    case DEBUG_IRRADIANCE_CACHE_SURFELS_IRRADIANCE:
      out_color = vec4(surfel.radiance, 1.0);
      break;
  }

  /* Display surfels as circles. */
  if (distance(P, surfel.position) > surfel_radius) {
    discard;
    return;
  }

  /* Display backfacing surfels with a transparent checkerboard grid. */
  if (!gl_FrontFacing) {
    ivec2 grid_uv = ivec2(gl_FragCoord) / 5;
    if ((grid_uv.x + grid_uv.y) % 2 == 0) {
      discard;
      return;
    }
  }
}
