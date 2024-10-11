
set(INC_GLSL
  # For shader includes and stubs
  ../../engines/eevee_next
  ../../engines/eevee_next/shaders
  ../../engines/workbench/shaders
  ../../engines/workbench
  ../../engines/overlay/shaders
  ../../engines/overlay
  ../../engines/select
  ../../intern/shaders
  ../../intern
  ../../../gpu/shaders/common
  ../../../gpu/shaders
  ../../../gpu/intern
  ../../../gpu/
)


cmake_policy(SET CMP0119 NEW)
set_source_files_properties(draw_debug_print_display_vert.glsl PROPERTIES LANGUAGE CXX)
add_executable(shader_check draw_debug_print_display_vert.glsl)
set_target_properties(shader_check PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(shader_check PUBLIC ${INC_GLSL})
