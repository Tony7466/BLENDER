#pragma BLENDER_REQUIRE(common_view_clipping_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

bool is_edge_sharpness_visible(float wd)
{
  return wd <= wireStepParam;
}

vec3 hsv_to_rgb(vec3 hsv)
{
  vec3 nrgb = abs(hsv.x * 6.0 - vec3(3.0, 2.0, 4.0)) * vec3(1, -1, -1) + vec3(-1, 2, 2);
  nrgb = clamp(nrgb, 0.0, 1.0);
  return ((nrgb - 1.0) * hsv.y + 1.0) * hsv.z;
}

void main()
{
  bool no_attr = all(equal(nor, vec3(0)));
  vec3 wnor = no_attr ? drw_view.viewinv[2].xyz : normalize(normal_object_to_world(nor));
  vec3 wpos = point_object_to_world(pos);

  if (isHair) {
    mat4 obmat = hairDupliMatrix;
    wpos = (obmat * vec4(pos, 1.0)).xyz;
    wnor = -normalize(mat3(obmat) * nor);
  }

  bool is_persp = (drw_view.winmat[3][3] == 0.0);
  vec3 V = (is_persp) ? normalize(drw_view.viewinv[3].xyz - wpos) : drw_view.viewinv[2].xyz;

  float facing = dot(wnor, V);

  gl_Position = point_world_to_ndc(wpos);

#ifndef CUSTOM_DEPTH_BIAS
  float facing_ratio = clamp(1.0 - facing * facing, 0.0, 1.0);
  float flip = sign(facing);           /* Flip when not facing the normal (i.e.: backfacing). */
  float curvature = (1.0 - wd * 0.75); /* Avoid making things worse for curvy areas. */
  vec3 wofs = wnor * (facing_ratio * curvature * flip);
  wofs = normal_world_to_view(wofs);

  /* Push vertex half a pixel (maximum) in normal direction. */
  gl_Position.xy += wofs.xy * sizeViewportInv * gl_Position.w;

  /* Push the vertex towards the camera. Helps a bit. */
  gl_Position.z -= facing_ratio * curvature * 1.0e-6 * gl_Position.w;
#endif

  /* Convert to screen position [0..sizeVp]. */
  edgeStart = ((gl_Position.xy / gl_Position.w) * 0.5 + 0.5) * sizeViewport.xy;

#ifndef SELECT_EDGES
  edgePos = edgeStart;
#else 
    /* HACK: to avoid losing sub-pixel object in selections, we add a bit of randomness to the
   * wire to at least create one fragment that will pass the occlusion query. */
  gl_Position.xy += sizeViewportInv * gl_Position.w * ((gl_VertexID % 2 == 0) ? -1.0 : 1.0);
#endif
  
  /* Cull flat edges below threshold. */
  if (!no_attr && !is_edge_sharpness_visible(wd)) {
    edgeStart = vec2(-1.0);
  }

  int flag = int(abs(ObjectInfo.w));
  bool is_selected = (flag & DRW_BASE_SELECTED) != 0;
  bool is_active = (flag & DRW_BASE_ACTIVE) != 0;
  bool selected_coloring = is_selected && useColoring;

  /* Base Color */
  vec3 wire_col, rim_col, no_fresnel_col, fresnel_col;

  if (isRandomColor) { /* Dim random color. */
    float hue = ObjectInfo.z;
    vec3 hsv = vec3(hue, 0.75, 0.8);
    wire_col = hsv_to_rgb(hsv);  
  }
  else { /* Initialize variable. */
    wire_col = ObjectColor.rgb;
  }
  wire_col = (isSingleColor) ? colorWire.rgb : wire_col;

  /* Fresnel */
  no_fresnel_col = wire_col;
  rim_col = wire_col;

  /* "Normalize" color. */
  wire_col += 1e-4; /* Avoid division by 0. */
  float brightness = max(wire_col.x, max(wire_col.y, wire_col.z));
  wire_col *= 0.5 / brightness;
  rim_col += 0.75;

  facing = clamp(abs(facing), 0.0, 1.0);

  /* Do interpolation in a non-linear space to have a better visual result. */
  rim_col = pow(rim_col, vec3(1.0 / 2.2));
  wire_col = pow(wire_col, vec3(1.0 / 2.2));
  fresnel_col = mix(rim_col, wire_col, 0.35);
  fresnel_col = mix(rim_col, fresnel_col, facing);
  fresnel_col = pow(fresnel_col, vec3(2.2));

  finalColor.rgb = mix(no_fresnel_col.rgb, fresnel_col, fresnelMix);

  /* Selection Color */
  if (selected_coloring) {
    if (isTransform) {
      finalColor.rgb = colorTransform.rgb;
    }
    else if (is_active) {
      finalColor.rgb = colorActive.rgb;
    }
    else {
      finalColor.rgb = colorSelect.rgb;
    }
  }

  finalColor.a = wireOpacity;
  finalColor.rgb *= wireOpacity;
  view_clipping_distances(wpos);
}
