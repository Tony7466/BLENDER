
void main()
{
  ivec3 froxel = ivec3(gl_GlobalInvocationID);

  if (any(greaterThanEqual(froxel, volumes_buf.tex_size))) {
    return;
  }

  vec4 clear = vec4(0.0);

  imageStore(out_scattering_img, froxel, clear);
  imageStore(out_extinction_img, froxel, clear);
  imageStore(out_emissive_img, froxel, clear);
  imageStore(out_phase_img, froxel, clear);
}
