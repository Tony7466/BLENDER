
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
  pointcloud_get_pos_and_nor(interp.P, interp.N);

#ifdef MAT_VELOCITY
  /* TODO(Miguel Pozo) */
#endif

  init_globals();
  attrib_load();

  interp.P += nodetree_displacement();

  gl_Position = point_world_to_ndc(interp.P);
}
