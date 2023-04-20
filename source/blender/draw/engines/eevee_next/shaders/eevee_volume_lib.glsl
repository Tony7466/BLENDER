#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_light_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_lib.glsl)

/* Based on Frosbite Unified Volumetric.
 * https://www.ea.com/frostbite/news/physically-based-unified-volumetric-rendering-in-frostbite */

/* Volume slice to view space depth. */
float volume_z_to_view_z(float z)
{
  float3 depth_params = volumes_buf.depth_param;
  if (ProjectionMatrix[3][3] == 0.0) {
    /* Exponential distribution */
    return (exp2(z / depth_params.z) - depth_params.x) / depth_params.y;
  }
  else {
    /* Linear distribution */
    return mix(depth_params.x, depth_params.y, z);
  }
}

float view_z_to_volume_z(float depth)
{
  float3 depth_params = volumes_buf.depth_param;
  if (ProjectionMatrix[3][3] == 0.0) {
    /* Exponential distribution */
    return depth_params.z * log2(depth * depth_params.y + depth_params.x);
  }
  else {
    /* Linear distribution */
    return (depth - depth_params.x) * depth_params.z;
  }
}

/* Volume texture normalized coordinates to NDC (special range [0, 1]). */
vec3 volume_to_ndc(vec3 coord)
{
  coord.z = volume_z_to_view_z(coord.z);
  coord.z = get_depth_from_view_z(coord.z);
  coord.xy /= volumes_buf.coord_scale;
  return coord;
}

vec3 ndc_to_volume(vec3 coord)
{
  /** NOTE: Keep in sync with to_global_grid_coords. */
  coord.z = get_view_z_from_depth(coord.z);
  coord.z = view_z_to_volume_z(coord.z);
  coord.xy *= volumes_buf.coord_scale;
  return coord;
}

float phase_function_isotropic()
{
  return 1.0 / (4.0 * M_PI);
}

float phase_function(vec3 v, vec3 l, float g)
{
  /* Henyey-Greenstein */
  float cos_theta = dot(v, l);
  g = clamp(g, -1.0 + 1e-3, 1.0 - 1e-3);
  float sqr_g = g * g;
  return (1 - sqr_g) / max(1e-8, 4.0 * M_PI * pow(1 + sqr_g - 2 * g * cos_theta, 3.0 / 2.0));
}

vec3 light_volume(LightData ld, vec4 l_vector)
{
  float power = 1.0;
  if (ld.type != LIGHT_SUN) {

    float volume_radius_squared = ld.radius_squared;
    float light_clamp = volumes_buf.light_clamp;
    if (light_clamp != 0.0) {
      /* 0.0 light clamp means it's disabled. */
      float max_power = max_v3(ld.color) * ld.volume_power;
      if (max_power > 0.0) {
        /* The limit of the power attenuation function when the distance to the light goes to 0 is
         * `2 / r^2` where r is the light radius. We need to find the right radius that emits at
         * most the volume light upper bound. Inverting the function we get: */
        float min_radius_squared = 1.0f / (0.5f * light_clamp / max_power);
        /* Square it here to avoid a multiplication inside the shader. */
        volume_radius_squared = max(volume_radius_squared, min_radius_squared);
      }
    }

    /**
     * Using "Point Light Attenuation Without Singularity" from Cem Yuksel
     * http://www.cemyuksel.com/research/pointlightattenuation/pointlightattenuation.pdf
     * http://www.cemyuksel.com/research/pointlightattenuation/
     */
    float d = l_vector.w;
    float d_sqr = sqr(d);
    float r_sqr = volume_radius_squared;

    /* Using reformulation that has better numerical precision. */
    power = 2.0 / (d_sqr + r_sqr + d * sqrt(d_sqr + r_sqr));

    if (ld.type == LIGHT_RECT || ld.type == LIGHT_ELLIPSE) {
      /* Modulate by light plane orientation / solid angle. */
      power *= saturate(dot(ld._back, l_vector.xyz / l_vector.w));
    }
  }
  return ld.color * ld.volume_power * power;
}

vec3 light_volume_light_vector(LightData ld, vec3 P)
{
  if (ld.type == LIGHT_SUN) {
    return ld._back;
  }
  else if (ld.type == LIGHT_RECT || ld.type == LIGHT_ELLIPSE) {
    vec3 L = P - ld._position;
    vec2 closest_point = vec2(dot(ld._right, L), dot(ld._up, L));
    vec2 max_pos = vec2(ld._area_size_x, ld._area_size_y);
    closest_point /= max_pos;

    if (ld.type == LIGHT_ELLIPSE) {
      closest_point /= max(1.0, length(closest_point));
    }
    else {
      closest_point = clamp(closest_point, -1.0, 1.0);
    }
    closest_point *= max_pos;

    vec3 L_prime = ld._right * closest_point.x + ld._up * closest_point.y;
    return L_prime - L;
  }
  else {
    return ld._position - P;
  }
}

#define VOLUMETRIC_SHADOW_MAX_STEP 128.0

vec3 participating_media_extinction(vec3 wpos, sampler3D volume_extinction)
{
  /* Waiting for proper volume shadowmaps and out of frustum shadow map. */
  vec3 ndc = project_point(ProjectionMatrix, transform_point(ViewMatrix, wpos));
  vec3 volume_co = ndc_to_volume(ndc * 0.5 + 0.5);

  /* Let the texture be clamped to edge. This reduce visual glitches. */
  return texture(volume_extinction, volume_co).rgb;
}

vec3 light_volume_shadow(LightData ld, vec3 ray_wpos, vec4 l_vector, sampler3D volume_extinction)
{
  /* TODO (Miguel Pozo) */
#if 0 && defined(VOLUME_SHADOW)
  /* If light is shadowed, use the shadow vector, if not, reuse the light vector. */
  if (volumes_buf.use_soft_shadows && ld.shadowid >= 0.0) {
    ShadowData sd = shadows_data[int(ld.shadowid)];

    if (ld.type == LIGHT_SUN) {
      l_vector.xyz = shadows_cascade_data[int(sd.sh_data_index)].sh_shadow_vec;
      /* No need for length, it is recomputed later. */
    }
    else {
      l_vector.xyz = shadows_cube_data[int(sd.sh_data_index)].position.xyz - ray_wpos;
      l_vector.w = length(l_vector.xyz);
    }
  }

  /* Heterogeneous volume shadows */
  float dd = l_vector.w / volumes_buf.shadow_steps;
  vec3 L = l_vector.xyz / volumes_buf.shadow_steps;

  if (ld.type == LIGHT_SUN) {
    /* For sun light we scan the whole frustum. So we need to get the correct endpoints. */
    vec3 ndcP = project_point(ProjectionMatrix, transform_point(ViewMatrix, ray_wpos));
    vec3 ndcL = project_point(ProjectionMatrix,
                              transform_point(ViewMatrix, ray_wpos + l_vector.xyz)) -
                ndcP;

    vec3 frustum_isect = ndcP + ndcL * line_unit_box_intersect_dist_safe(ndcP, ndcL);

    vec4 L_hom = ViewMatrixInverse * (ProjectionMatrixInverse * vec4(frustum_isect, 1.0));
    L = (L_hom.xyz / L_hom.w) - ray_wpos;
    L /= volumes_buf.shadow_steps;
    dd = length(L);
  }

  vec3 shadow = vec3(1.0);
  for (float s = 1.0; s < VOLUMETRIC_SHADOW_MAX_STEP && s <= volumes_buf.shadow_steps; s += 1.0) {
    vec3 pos = ray_wpos + L * s;
    vec3 s_extinction = participating_media_extinction(pos, volume_extinction);
    shadow *= exp(-s_extinction * dd);
  }
  return shadow;
#else
  return vec3(1.0);
#endif /* VOLUME_SHADOW */
}

vec3 irradiance_volumetric(vec3 wpos)
{
#ifdef IRRADIANCE_HL2
  IrradianceData ir_data = load_irradiance_cell(0, vec3(1.0));
  vec3 irradiance = ir_data.cubesides[0] + ir_data.cubesides[1] + ir_data.cubesides[2];
  ir_data = load_irradiance_cell(0, vec3(-1.0));
  irradiance += ir_data.cubesides[0] + ir_data.cubesides[1] + ir_data.cubesides[2];
  irradiance *= 0.16666666; /* 1/6 */
  return irradiance;
#else
  return vec3(0.0);
#endif
}

void volumetric_resolve(vec2 frag_uvs,
                        float frag_depth,
                        out vec3 transmittance,
                        out vec3 scattering)
{
  vec3 coord = ndc_to_volume(vec3(frag_uvs, frag_depth));

  scattering = texture(inScattering, coord).rgb;
  transmittance = texture(inTransmittance, coord).rgb;
}
