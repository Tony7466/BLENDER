
/* This shader is used to add default values to the volume accum textures.
 * so it looks similar (transmittance = 1, scattering = 0) */
void main()
{
  out_radiance = vec4(0.0);
  out_transmittance = vec4(1.0);
}
