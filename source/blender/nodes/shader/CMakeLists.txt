# SPDX-FileCopyrightText: 2021 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  .
  ..
  ../intern
  ../../blenkernel
  ../../blentranslation
  ../../depsgraph
  ../../editors/include
  ../../functions
  ../../gpu
  ../../imbuf
  ../../makesrna
  ../../render
  ../../windowmanager
  ../../../../intern/sky/include
  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

if(WITH_HYDRA)
  list(APPEND INC
    ../../io/usd
    ../../../../intern/clog
)
  add_definitions(-DWITH_HYDRA)
endif()

set(INC_SYS
)

set(SRC
  nodes/node_shader_add_shader.cc
  nodes/node_shader_ambient_occlusion.cc
  nodes/node_shader_attribute.cc
  nodes/node_shader_background.cc
  nodes/node_shader_bevel.cc
  nodes/node_shader_blackbody.cc
  nodes/node_shader_brightness.cc
  nodes/node_shader_bsdf_diffuse.cc
  nodes/node_shader_bsdf_glass.cc
  nodes/node_shader_bsdf_glossy.cc
  nodes/node_shader_bsdf_hair.cc
  nodes/node_shader_bsdf_hair_principled.cc
  nodes/node_shader_bsdf_principled.cc
  nodes/node_shader_bsdf_refraction.cc
  nodes/node_shader_bsdf_sheen.cc
  nodes/node_shader_bsdf_toon.cc
  nodes/node_shader_bsdf_translucent.cc
  nodes/node_shader_bsdf_transparent.cc
  nodes/node_shader_bump.cc
  nodes/node_shader_camera.cc
  nodes/node_shader_clamp.cc
  nodes/node_shader_color_ramp.cc
  nodes/node_shader_common.cc
  nodes/node_shader_curves.cc
  nodes/node_shader_displacement.cc
  nodes/node_shader_eevee_specular.cc
  nodes/node_shader_emission.cc
  nodes/node_shader_fresnel.cc
  nodes/node_shader_gamma.cc
  nodes/node_shader_geometry.cc
  nodes/node_shader_hair_info.cc
  nodes/node_shader_holdout.cc
  nodes/node_shader_hueSatVal.cc
  nodes/node_shader_ies_light.cc
  nodes/node_shader_invert.cc
  nodes/node_shader_layer_weight.cc
  nodes/node_shader_light_falloff.cc
  nodes/node_shader_light_path.cc
  nodes/node_shader_map_range.cc
  nodes/node_shader_mapping.cc
  nodes/node_shader_math.cc
  nodes/node_shader_mix.cc
  nodes/node_shader_mix_rgb.cc
  nodes/node_shader_mix_shader.cc
  nodes/node_shader_normal.cc
  nodes/node_shader_normal_map.cc
  nodes/node_shader_object_info.cc
  nodes/node_shader_output_aov.cc
  nodes/node_shader_output_light.cc
  nodes/node_shader_output_linestyle.cc
  nodes/node_shader_output_material.cc
  nodes/node_shader_output_world.cc
  nodes/node_shader_particle_info.cc
  nodes/node_shader_point_info.cc
  nodes/node_shader_rgb.cc
  nodes/node_shader_rgb_to_bw.cc
  nodes/node_shader_script.cc
  nodes/node_shader_sepcomb_color.cc
  nodes/node_shader_sepcomb_hsv.cc
  nodes/node_shader_sepcomb_rgb.cc
  nodes/node_shader_sepcomb_xyz.cc
  nodes/node_shader_shader_to_rgb.cc
  nodes/node_shader_squeeze.cc
  nodes/node_shader_subsurface_scattering.cc
  nodes/node_shader_tangent.cc
  nodes/node_shader_tex_brick.cc
  nodes/node_shader_tex_checker.cc
  nodes/node_shader_tex_coord.cc
  nodes/node_shader_tex_environment.cc
  nodes/node_shader_tex_gradient.cc
  nodes/node_shader_tex_image.cc
  nodes/node_shader_tex_magic.cc
  nodes/node_shader_tex_musgrave.cc
  nodes/node_shader_tex_noise.cc
  nodes/node_shader_tex_pointdensity.cc
  nodes/node_shader_tex_sky.cc
  nodes/node_shader_tex_voronoi.cc
  nodes/node_shader_tex_wave.cc
  nodes/node_shader_tex_white_noise.cc
  nodes/node_shader_uv_along_stroke.cc
  nodes/node_shader_uvmap.cc
  nodes/node_shader_value.cc
  nodes/node_shader_vector_displacement.cc
  nodes/node_shader_vector_math.cc
  nodes/node_shader_vector_rotate.cc
  nodes/node_shader_vector_transform.cc
  nodes/node_shader_vertex_color.cc
  nodes/node_shader_volume_absorption.cc
  nodes/node_shader_volume_info.cc
  nodes/node_shader_volume_principled.cc
  nodes/node_shader_volume_scatter.cc
  nodes/node_shader_wavelength.cc
  nodes/node_shader_wireframe.cc

  node_shader_register.cc
  node_shader_tree.cc
  node_shader_util.cc

  node_shader_register.hh
  node_shader_util.hh
)

set(LIB
  PRIVATE bf::blenlib
  PRIVATE bf::dna
  bf_functions
  PRIVATE bf::intern::guardedalloc
  bf_intern_sky
  bf_nodes
)

if(WITH_MATERIALX)
  list(APPEND SRC
    materialx/material.cc
    materialx/nodes/add_shader.cc
    materialx/nodes/blackbody.cc
    materialx/nodes/brightness.cc
    materialx/nodes/bsdf_diffuse.cc
    materialx/nodes/bsdf_glass.cc
    materialx/nodes/bsdf_glossy.cc
    materialx/nodes/bsdf_principled.cc
    materialx/nodes/bsdf_refraction.cc
    materialx/nodes/bsdf_sheen.cc
    materialx/nodes/bsdf_toon.cc
    materialx/nodes/bsdf_translucent.cc
    materialx/nodes/bsdf_transparent.cc
    materialx/nodes/clamp.cc
    materialx/nodes/color_ramp.cc
    materialx/nodes/curves.cc
    materialx/nodes/gamma.cc
    materialx/nodes/emission.cc
    materialx/nodes/huesatval.cc
    materialx/nodes/invert.cc
    materialx/nodes/light_falloff.cc
    materialx/nodes/light_path.cc
    materialx/nodes/map_range.cc
    materialx/nodes/math.cc
    materialx/nodes/mix_rgb.cc
    materialx/nodes/mix_shader.cc
    materialx/nodes/node_item.cc
    materialx/nodes/node_parser.cc
    materialx/nodes/normal_map.cc
    materialx/nodes/output_material.cc
    materialx/nodes/rgb_to_bw.cc
    materialx/nodes/sepcomb_color.cc
    materialx/nodes/sepcomb_xyz.cc
    materialx/nodes/subsurface_scattering.cc
    materialx/nodes/tex_checker.cc
    materialx/nodes/tex_environment.cc
    materialx/nodes/tex_image.cc
    materialx/nodes/tex_noise.cc
    materialx/nodes/vector_math.cc
    materialx/nodes/wavelength.cc

    materialx/material.h
    materialx/nodes/node_item.h
    materialx/nodes/node_parser.h
    materialx/nodes/output_material.h
  )
  list(APPEND LIB
    MaterialXCore
    MaterialXFormat
  )
endif()

if(WITH_FREESTYLE)
  add_definitions(-DWITH_FREESTYLE)
endif()

if(WITH_TBB)
  add_definitions(-DWITH_TBB)
  if(WIN32)
    # TBB includes Windows.h which will define min/max macros
    # that will collide with the stl versions.
    add_definitions(-DNOMINMAX)
  endif()
  list(APPEND INC_SYS
    ${TBB_INCLUDE_DIRS}
  )

  list(APPEND LIB
    ${TBB_LIBRARIES}
  )
endif()

blender_add_lib(bf_nodes_shader "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

if(WITH_UNITY_BUILD)
  set_target_properties(bf_nodes_shader PROPERTIES UNITY_BUILD ON)
  set_target_properties(bf_nodes_shader PROPERTIES UNITY_BUILD_BATCH_SIZE 10)
endif()

# RNA_prototypes.h
add_dependencies(bf_nodes_shader bf_rna)
