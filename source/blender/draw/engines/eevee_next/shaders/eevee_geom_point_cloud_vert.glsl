
#pragma BLENDER_REQUIRE(common_pointcloud_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_attributes_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_nodetree_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_surf_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_velocity_lib.glsl)

void main()
{
  DRW_VIEW_FROM_RESOURCE_ID;
#ifdef MAT_SHADOW
  shadow_interp.view_id = drw_view_id;
#endif

  init_interface();

  point_cloud_interp.ID = pointcloud_get_point_id();
  pointcloud_get_pos_and_radius(point_cloud_interp.position, point_cloud_interp.radius);

  mat3 facing_mat;
  {
    /* Same logic as pointcloud_get_facing_matrix(), but ensuring we use the actual camera matrices
     * so shadowmaps are rendered correctly. */
    facing_mat[2] = camera_buf.persmat[3][3] == 0.0 ?
                        normalize(camera_buf.viewinv[3].xyz - point_cloud_interp.position) :
                        camera_buf.viewinv[2].xyz;
    facing_mat[1] = normalize(cross(camera_buf.viewinv[0].xyz, facing_mat[2]));
    facing_mat[0] = cross(facing_mat[1], facing_mat[2]);
  }

  interp.N = pointcloud_get_normal(facing_mat);
  /* TODO(fclem): remove multiplication here. Here only for keeping the size correct for now. */
  float radius = point_cloud_interp.radius * 0.01;
  interp.P = point_cloud_interp.position + interp.N * radius;

#ifdef MAT_VELOCITY
  vec3 prv, nxt;
  velocity_local_pos_get(point_cloud_interp.position, point_cloud_interp.ID, prv, nxt);
  /* FIXME(fclem): Evaluating before displacement avoid displacement being treated as motion but
   * ignores motion from animated displacement. Supporting animated displacement motion vectors
   * would require evaluating the nodetree multiple time with different nodetree UBOs evaluated at
   * different times, but also with different attributes (maybe we could assume static attribute at
   * least). */
  velocity_vertex(prv, point_cloud_interp.position, nxt, motion.prev, motion.next);
#endif

  init_globals();
  attrib_load();

  interp.P += nodetree_displacement();

  gl_Position = point_world_to_ndc(interp.P);
}
