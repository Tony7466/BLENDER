
void main()
{
  Surfel surfel = surfels_buf[surfel_index];

  vec4 radiance_bounce = gl_FrontFacing ? surfel.radiance_bounce_front :
                                          surfel.radiance_bounce_back;
  vec3 radiance = gl_FrontFacing ? surfel.radiance_front : surfel.radiance_back;

  switch (eDebugMode(debug_mode)) {
    default:
    case DEBUG_IRRADIANCE_CACHE_SURFELS_NORMAL:
      out_color = vec4(pow(surfel.normal * 0.5 + 0.5, vec3(2.2)), 0.0);
      break;
    case DEBUG_IRRADIANCE_CACHE_SURFELS_IRRADIANCE:
      out_color = (radiance_bounce.w > 0.0) ?
                      vec4(radiance + radiance_bounce.rgb / radiance_bounce.w, 0.0) :
                      vec4(radiance, 0.0);
      break;
  }

  /* Display surfels as circles. */
  if (distance(P, surfel.position) > surfel_radius) {
    discard;
    return;
  }
}
