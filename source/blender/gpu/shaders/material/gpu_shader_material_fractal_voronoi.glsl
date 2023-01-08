#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_common_math_utils.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_material_voronoi.glsl)

void fractal_voronoi_f1_3d(vec3 coord,
    float scale,
    float exponent,
    float randomness,
    float metric,
    float max_distance,
    out float max_amplitude,
    out float outDistance,
    out vec4 outColor,
    out vec3 outPosition)
{
    voronoi_f1_3d(coord,
        scale,
        exponent,
        randomness,
        metric,
        outDistance,
        vec4 outColor,
        vec3 outPosition);
}
