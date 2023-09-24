/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#ifndef SVM_CASE
#define SVM_CASE case node_type:
#endif

#ifndef NODE_HANDLER_CALL
#define NODE_HANDLER_CALL(...) __VA_ARGS__
#endif

#ifndef JUMP_NODE_HANDLER_CALL
#define JUMP_NODE_HANDLER_CALL(...) NODE_HANDLER_CALL(__VA_ARGS__)
#endif

#ifndef BREAK_CASE
#define BREAK_CASE() break;
#endif

  SVM_CASE(NODE_END)
  NODE_HANDLER_CALL(
  return;
  )
  SVM_CASE(NODE_SHADER_JUMP)
  {
    JUMP_NODE_HANDLER_CALL(
    if (type == SHADER_TYPE_SURFACE)
    {
      offset = node.y;
      /*SHADER_TYPE_SURFACE_HANDLER*/
    }
    else if (type == SHADER_TYPE_VOLUME)
    {
      offset = node.z;
      /*SHADER_TYPE_VOLUME_HANDLER*/
    }
    else if (type == SHADER_TYPE_DISPLACEMENT)
    {
      offset = node.w;
      /*SHADER_TYPE_DISPLACEMENT_HANDLER*/
    }
    else
      return;
    )
    BREAK_CASE();
  }
  SVM_CASE(NODE_CLOSURE_BSDF)
  NODE_HANDLER_CALL(
  offset = svm_node_closure_bsdf<node_feature_mask, type>(
      kg, sd, stack, closure_weight, node, path_flag, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_CLOSURE_EMISSION)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(EMISSION)
  {
    svm_node_closure_emission(sd, stack, closure_weight, node);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_CLOSURE_BACKGROUND)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(EMISSION)
  {
    svm_node_closure_background(sd, stack, closure_weight, node);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_CLOSURE_SET_WEIGHT)
  NODE_HANDLER_CALL(
  svm_node_closure_set_weight(sd, &closure_weight, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_CLOSURE_WEIGHT)
  NODE_HANDLER_CALL(
  svm_node_closure_weight(sd, stack, &closure_weight, node.y);
  )
  BREAK_CASE();
  SVM_CASE(NODE_EMISSION_WEIGHT)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(EMISSION)
  {
    svm_node_emission_weight(kg, sd, stack, &closure_weight, node);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_MIX_CLOSURE)
  NODE_HANDLER_CALL(
  svm_node_mix_closure(sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_JUMP_IF_ZERO)
  JUMP_NODE_HANDLER_CALL(
  if (stack_load_float(stack, node.z) <= 0.0f)
  {
    offset += node.y;
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_JUMP_IF_ONE)
  JUMP_NODE_HANDLER_CALL(
  if (stack_load_float(stack, node.z) >= 1.0f)
  {
    offset += node.y;
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_GEOMETRY)
  NODE_HANDLER_CALL(
  svm_node_geometry(kg, sd, stack, node.y, node.z);
  )
  BREAK_CASE();
  SVM_CASE(NODE_CONVERT)
  NODE_HANDLER_CALL(
  svm_node_convert(kg, sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_COORD)
  NODE_HANDLER_CALL(
  offset = svm_node_tex_coord(kg, sd, path_flag, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_VALUE_F)
  NODE_HANDLER_CALL(
  svm_node_value_f(kg, sd, stack, node.y, node.z);
  )
  BREAK_CASE();
  SVM_CASE(NODE_VALUE_V)
  NODE_HANDLER_CALL(
  offset = svm_node_value_v(kg, sd, stack, node.y, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_ATTR)
  NODE_HANDLER_CALL(
  svm_node_attr<node_feature_mask>(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_VERTEX_COLOR)
  NODE_HANDLER_CALL(
  svm_node_vertex_color(kg, sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_GEOMETRY_BUMP_DX)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP)
  {
    svm_node_geometry_bump_dx(kg, sd, stack, node.y, node.z);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_GEOMETRY_BUMP_DY)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP)
  {
    svm_node_geometry_bump_dy(kg, sd, stack, node.y, node.z);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_SET_DISPLACEMENT)
  NODE_HANDLER_CALL(
  svm_node_set_displacement<node_feature_mask>(kg, sd, stack, node.y);
  )
  BREAK_CASE();
  SVM_CASE(NODE_DISPLACEMENT)
  NODE_HANDLER_CALL(
  svm_node_displacement<node_feature_mask>(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_VECTOR_DISPLACEMENT)
  NODE_HANDLER_CALL(
  offset = svm_node_vector_displacement<node_feature_mask>(kg, sd, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_IMAGE)
  NODE_HANDLER_CALL(
  offset = svm_node_tex_image(kg, sd, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_IMAGE_BOX)
  NODE_HANDLER_CALL(
  svm_node_tex_image_box(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_NOISE)
  NODE_HANDLER_CALL(
  offset = svm_node_tex_noise(kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_SET_BUMP)
  NODE_HANDLER_CALL(
  svm_node_set_bump<node_feature_mask>(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_ATTR_BUMP_DX)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP)
  {
    svm_node_attr_bump_dx(kg, sd, stack, node);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_ATTR_BUMP_DY)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP)
  {
    svm_node_attr_bump_dy(kg, sd, stack, node);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_VERTEX_COLOR_BUMP_DX)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP)
  {
    svm_node_vertex_color_bump_dx(kg, sd, stack, node.y, node.z, node.w);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_VERTEX_COLOR_BUMP_DY)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP)
  {
    svm_node_vertex_color_bump_dy(kg, sd, stack, node.y, node.z, node.w);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_COORD_BUMP_DX)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP)
  {
    offset = svm_node_tex_coord_bump_dx(kg, sd, path_flag, stack, node, offset);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_COORD_BUMP_DY)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP)
  {
    offset = svm_node_tex_coord_bump_dy(kg, sd, path_flag, stack, node, offset);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_CLOSURE_SET_NORMAL)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP)
  {
    svm_node_set_normal(kg, sd, stack, node.y, node.z);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_ENTER_BUMP_EVAL)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP_STATE)
  {
    svm_node_enter_bump_eval(kg, sd, stack, node.y);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_LEAVE_BUMP_EVAL)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(BUMP_STATE)
  {
    svm_node_leave_bump_eval(kg, sd, stack, node.y);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_HSV)
  NODE_HANDLER_CALL(
  svm_node_hsv(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_CLOSURE_HOLDOUT)
  NODE_HANDLER_CALL(
  svm_node_closure_holdout(sd, stack, closure_weight, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_FRESNEL)
  NODE_HANDLER_CALL(
  svm_node_fresnel(sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_LAYER_WEIGHT)
  NODE_HANDLER_CALL(
  svm_node_layer_weight(sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_CLOSURE_VOLUME)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(VOLUME)
  {
    svm_node_closure_volume<type>(kg, sd, stack, closure_weight, node);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_PRINCIPLED_VOLUME)
  NODE_HANDLER_CALL(
  IF_KERNEL_NODES_FEATURE(VOLUME)
  {
    offset = svm_node_principled_volume<type>(kg, sd, stack, closure_weight, node, path_flag, offset);
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_MATH)
  NODE_HANDLER_CALL(
  svm_node_math(kg, sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_VECTOR_MATH)
  NODE_HANDLER_CALL(
  offset = svm_node_vector_math(kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_RGB_RAMP)
  NODE_HANDLER_CALL(
  offset = svm_node_rgb_ramp(kg, sd, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_GAMMA)
  NODE_HANDLER_CALL(
  svm_node_gamma(sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_BRIGHTCONTRAST)
  NODE_HANDLER_CALL(
  svm_node_brightness(sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_LIGHT_PATH)
  NODE_HANDLER_CALL(
  svm_node_light_path<node_feature_mask>(kg, state, sd, stack, node.y, node.z, path_flag);
  )
  BREAK_CASE();
  SVM_CASE(NODE_OBJECT_INFO)
  NODE_HANDLER_CALL(
  svm_node_object_info(kg, sd, stack, node.y, node.z);
  )
  BREAK_CASE();
  SVM_CASE(NODE_PARTICLE_INFO)
  NODE_HANDLER_CALL(
  svm_node_particle_info(kg, sd, stack, node.y, node.z);
  )
  BREAK_CASE();
#if defined(__HAIR__)
  SVM_CASE(NODE_HAIR_INFO)
  NODE_HANDLER_CALL(
  svm_node_hair_info(kg, sd, stack, node.y, node.z);
  )
  BREAK_CASE();
#endif
#if defined(__POINTCLOUD__)
  SVM_CASE(NODE_POINT_INFO)
  NODE_HANDLER_CALL(
  svm_node_point_info(kg, sd, stack, node.y, node.z);
  )
  BREAK_CASE();
#endif
  SVM_CASE(NODE_TEXTURE_MAPPING)
  NODE_HANDLER_CALL(
  offset = svm_node_texture_mapping(kg, sd, stack, node.y, node.z, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_MAPPING)
  NODE_HANDLER_CALL(
  svm_node_mapping(kg, sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_MIN_MAX)
  NODE_HANDLER_CALL(
  offset = svm_node_min_max(kg, sd, stack, node.y, node.z, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_CAMERA)
  NODE_HANDLER_CALL(
  svm_node_camera(kg, sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_ENVIRONMENT)
  NODE_HANDLER_CALL(
  svm_node_tex_environment(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_SKY)
  NODE_HANDLER_CALL(
  offset = svm_node_tex_sky(kg, sd, path_flag, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_GRADIENT)
  NODE_HANDLER_CALL(
  svm_node_tex_gradient(sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_VORONOI)
  NODE_HANDLER_CALL(
  offset = svm_node_tex_voronoi<node_feature_mask>(
      kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_MUSGRAVE)
  NODE_HANDLER_CALL(
  offset = svm_node_tex_musgrave(kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_WAVE)
  NODE_HANDLER_CALL(
  offset = svm_node_tex_wave(kg, sd, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_MAGIC)
  NODE_HANDLER_CALL(
  offset = svm_node_tex_magic(kg, sd, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_CHECKER)
  NODE_HANDLER_CALL(
  svm_node_tex_checker(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_BRICK)
  NODE_HANDLER_CALL(
  offset = svm_node_tex_brick(kg, sd, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TEX_WHITE_NOISE)
  NODE_HANDLER_CALL(
  svm_node_tex_white_noise(kg, sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_NORMAL)
  NODE_HANDLER_CALL(
  offset = svm_node_normal(kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_LIGHT_FALLOFF)
  NODE_HANDLER_CALL(
  svm_node_light_falloff(sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_IES)
  NODE_HANDLER_CALL(
  svm_node_ies(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_CURVES)
  NODE_HANDLER_CALL(
  offset = svm_node_curves(kg, sd, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_FLOAT_CURVE)
  NODE_HANDLER_CALL(
  offset = svm_node_curve(kg, sd, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_TANGENT)
  NODE_HANDLER_CALL(
  svm_node_tangent(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_NORMAL_MAP)
  NODE_HANDLER_CALL(
  svm_node_normal_map(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_INVERT)
  NODE_HANDLER_CALL(
  svm_node_invert(sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_MIX)
  NODE_HANDLER_CALL(
  offset = svm_node_mix(kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_SEPARATE_COLOR)
  NODE_HANDLER_CALL(
  svm_node_separate_color(kg, sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_COMBINE_COLOR)
  NODE_HANDLER_CALL(
  svm_node_combine_color(kg, sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_SEPARATE_VECTOR)
  NODE_HANDLER_CALL(
  svm_node_separate_vector(sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_COMBINE_VECTOR)
  NODE_HANDLER_CALL(
  svm_node_combine_vector(sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_SEPARATE_HSV)
  NODE_HANDLER_CALL(
  offset = svm_node_separate_hsv(kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_COMBINE_HSV)
  NODE_HANDLER_CALL(
  offset = svm_node_combine_hsv(kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_VECTOR_ROTATE)
  NODE_HANDLER_CALL(
  svm_node_vector_rotate(sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_VECTOR_TRANSFORM)
  NODE_HANDLER_CALL(
  svm_node_vector_transform(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_WIREFRAME)
  NODE_HANDLER_CALL(
  svm_node_wireframe(kg, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_WAVELENGTH)
  NODE_HANDLER_CALL(
  svm_node_wavelength(kg, sd, stack, node.y, node.z);
  )
  BREAK_CASE();
  SVM_CASE(NODE_BLACKBODY)
  NODE_HANDLER_CALL(
  svm_node_blackbody(kg, sd, stack, node.y, node.z);
  )
  BREAK_CASE();
  SVM_CASE(NODE_MAP_RANGE)
  NODE_HANDLER_CALL(
  offset = svm_node_map_range(kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_VECTOR_MAP_RANGE)
  NODE_HANDLER_CALL(
  offset = svm_node_vector_map_range(kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_CLAMP)
  NODE_HANDLER_CALL(
  offset = svm_node_clamp(kg, sd, stack, node.y, node.z, node.w, offset);
  )
  BREAK_CASE();
#ifdef __SHADER_RAYTRACE__
  SVM_CASE(NODE_BEVEL)
  NODE_HANDLER_CALL(
  svm_node_bevel<node_feature_mask>(kg, state, sd, stack, node);
  )
  BREAK_CASE();
  SVM_CASE(NODE_AMBIENT_OCCLUSION)
  NODE_HANDLER_CALL(
  svm_node_ao<node_feature_mask>(kg, state, sd, stack, node);
  )
  BREAK_CASE();
#endif

  SVM_CASE(NODE_TEX_VOXEL)
  NODE_HANDLER_CALL(
  offset = svm_node_tex_voxel<node_feature_mask>(kg, sd, stack, node, offset);
  )
  BREAK_CASE();
  SVM_CASE(NODE_AOV_START)
  NODE_HANDLER_CALL(
  if (!svm_node_aov_check(path_flag, render_buffer)) {
    return;
  }
  )
  BREAK_CASE();
  SVM_CASE(NODE_AOV_COLOR)
  NODE_HANDLER_CALL(
  svm_node_aov_color<node_feature_mask>(kg, state, sd, stack, node, render_buffer);
  )
  BREAK_CASE();
  SVM_CASE(NODE_AOV_VALUE)
  NODE_HANDLER_CALL(
  svm_node_aov_value<node_feature_mask>(kg, state, sd, stack, node, render_buffer);
  )
  BREAK_CASE();
  SVM_CASE(NODE_MIX_COLOR)
  NODE_HANDLER_CALL(
  svm_node_mix_color(sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_MIX_FLOAT)
  NODE_HANDLER_CALL(
  svm_node_mix_float(sd, stack, node.y, node.z, node.w);
  )
  BREAK_CASE();
  SVM_CASE(NODE_MIX_VECTOR)
  NODE_HANDLER_CALL(
  svm_node_mix_vector(sd, stack, node.y, node.z);
  )
  BREAK_CASE();
  SVM_CASE(NODE_MIX_VECTOR_NON_UNIFORM)
  NODE_HANDLER_CALL(
  svm_node_mix_vector_non_uniform(sd, stack, node.y, node.z);
  )
  BREAK_CASE();
