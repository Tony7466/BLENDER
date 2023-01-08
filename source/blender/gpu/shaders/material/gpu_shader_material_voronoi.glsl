#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_common_math_utils.glsl)

voronoi_f1_3d(vec3 coord,
    float scale,
    float exponent,
    float randomness,
    float metric,
    out float outDistance,
    out vec4 outColor,
    out vec3 outPosition)
{
    vec3 scaledCoord = coord * scale;
    vec3 cellPosition = floor(scaledCoord);
    vec3 localPosition = scaledCoord - cellPosition;

    float minDistance = 8.0;
    vec3 targetOffset, targetPosition;
    for (int k = -1; k <= 1; k++) {
        for (int j = -1; j <= 1; j++) {
            for (int i = -1; i <= 1; i++) {
                vec3 cellOffset = vec3(i, j, k);
                vec3 pointPosition = cellOffset +
                    hash_vec3_to_vec3(cellPosition + cellOffset) * randomness;
                float distanceToPoint = voronoi_distance(pointPosition, localPosition, metric, exponent);
                if (distanceToPoint < minDistance) {
                    targetOffset = cellOffset;
                    minDistance = distanceToPoint;
                    targetPosition = pointPosition;
                }
            }
        }
    }
    outDistance = minDistance;
    outColor.xyz = hash_vec3_to_vec3(cellPosition + targetOffset);
    outPosition = safe_divide(targetPosition + cellPosition, scale);
}
