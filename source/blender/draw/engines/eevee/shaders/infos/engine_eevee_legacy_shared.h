/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */
#ifndef EEVEE_SHADER_SHARED_H
#define EEVEE_SHADER_SHARED_H
#ifndef GPU_SHADER
typedef struct CommonUniformBlock CommonUniformBlock;
#endif

#ifdef GPU_SHADER
/* Catch for non-create info case. */
#  ifndef BLI_STATIC_ASSERT_ALIGN
#    define BLI_STATIC_ASSERT_ALIGN(type, alignment)
#  endif
#endif

/* NOTE: AMD-based macOS platforms experience performance and correctness issues with EEVEE
 * material closure evaluation. Using singular closure evaluation, rather than the compound
 * function calls reduces register overflow, by limiting the simultaneous number of live
 * registers used by the virtual GPU function stack. */
#if (defined(GPU_METAL) && defined(GPU_ATI))
#  define DO_SPLIT_CLOSURE_EVAL 1
#endif

/* NOTE: Due to compatibility issues between legacy Intel drivers that doesn't unroll macros
 * as expected we added an underscore before the attribute name to make a distinction between the
 * macro name and the variable it needs to access. This fails on MacOS/ATI resulting in failing to
 * render Eevee on those devices. Issue happens both on Metal and OpenGL backend.
 *
 * This is a quick fix to support both issues until we have a better solution or that Eevee is
 * replaced.
 */
#if (defined(OS_MAC) && defined(GPU_ATI))
#  define BLOCK_ATTR(name) name
#else
#  define BLOCK_ATTR(name) _##name
#endif

struct CommonUniformBlock {
  mat4 BLOCK_ATTR(pastViewProjectionMatrix);
  vec4 BLOCK_ATTR(hizUvScale); /* To correct mip level texel misalignment */
  /* Ambient Occlusion */
  vec4 BLOCK_ATTR(aoParameters)[2];
  /* Volumetric */
  ivec4 BLOCK_ATTR(volTexSize);
  vec4 BLOCK_ATTR(volDepthParameters); /* Parameters to the volume Z equation */
  vec4 BLOCK_ATTR(volInvTexSize);
  vec4 BLOCK_ATTR(volJitter);
  vec4 BLOCK_ATTR(volCoordScale); /* To convert volume uvs to screen uvs */
  float BLOCK_ATTR(volHistoryAlpha);
  float BLOCK_ATTR(volShadowSteps);
  bool BLOCK_ATTR(volUseLights);
  bool BLOCK_ATTR(volUseSoftShadows);
  /* Screen Space Reflections */
  vec4 BLOCK_ATTR(ssrParameters);
  float BLOCK_ATTR(ssrBorderFac);
  float BLOCK_ATTR(ssrMaxRoughness);
  float BLOCK_ATTR(ssrFireflyFac);
  float BLOCK_ATTR(ssrBrdfBias);
  bool BLOCK_ATTR(ssrToggle);
  bool BLOCK_ATTR(ssrefractToggle);
  /* SubSurface Scattering */
  float BLOCK_ATTR(sssJitterThreshold);
  bool BLOCK_ATTR(sssToggle);
  /* Specular */
  bool BLOCK_ATTR(specToggle);
  /* Lights */
  int BLOCK_ATTR(laNumLight);
  /* Probes */
  int BLOCK_ATTR(prbNumPlanar);
  int BLOCK_ATTR(prbNumRenderCube);
  int BLOCK_ATTR(prbNumRenderGrid);
  int BLOCK_ATTR(prbIrradianceVisSize);
  float BLOCK_ATTR(prbIrradianceSmooth);
  float BLOCK_ATTR(prbLodCubeMax);
  /* Misc */
  int BLOCK_ATTR(rayType);
  float BLOCK_ATTR(rayDepth);
  float BLOCK_ATTR(alphaHashOffset);
  float BLOCK_ATTR(alphaHashScale);
  vec4 BLOCK_ATTR(cameraUvScaleBias);
  vec4 BLOCK_ATTR(planarClipPlane);
};
BLI_STATIC_ASSERT_ALIGN(CommonUniformBlock, 16)

struct CubeData {
  vec4 position_type;
  vec4 attenuation_fac_type;
  mat4 influencemat;
  mat4 parallaxmat;
};
BLI_STATIC_ASSERT_ALIGN(CubeData, 16)

struct PlanarData {
  vec4 plane_equation;
  vec4 clip_vec_x_fade_scale;
  vec4 clip_vec_y_fade_bias;
  vec4 clip_edges;
  vec4 facing_scale_bias;
  mat4 reflectionmat; /* transform world space into reflection texture space */
  mat4 unused;
};
BLI_STATIC_ASSERT_ALIGN(PlanarData, 16)

struct GridData {
  mat4 localmat;
  ivec4 resolution_offset;
  vec4 ws_corner_atten_scale;     /* world space corner position */
  vec4 ws_increment_x_atten_bias; /* world space vector between 2 opposite cells */
  vec4 ws_increment_y_lvl_bias;
  vec4 ws_increment_z;
  vec4 vis_bias_bleed_range;
};
BLI_STATIC_ASSERT_ALIGN(GridData, 16)

struct ProbeBlock {
  CubeData BLOCK_ATTR(probes_data)[MAX_PROBE];
};
BLI_STATIC_ASSERT_ALIGN(ProbeBlock, 16)

struct GridBlock {
  GridData BLOCK_ATTR(grids_data)[MAX_GRID];
};
BLI_STATIC_ASSERT_ALIGN(GridBlock, 16)

struct PlanarBlock {
  PlanarData BLOCK_ATTR(planars_data)[MAX_PLANAR];
};
BLI_STATIC_ASSERT_ALIGN(PlanarBlock, 16)

#ifndef MAX_CASCADE_NUM
#  define MAX_CASCADE_NUM 4
#endif

struct ShadowData {
  vec4 near_far_bias_id;
  vec4 contact_shadow_data;
};
BLI_STATIC_ASSERT_ALIGN(ShadowData, 16)

struct ShadowCubeData {
  mat4 shadowmat;
  vec4 position;
};
BLI_STATIC_ASSERT_ALIGN(ShadowCubeData, 16)

struct ShadowCascadeData {
  mat4 shadowmat[MAX_CASCADE_NUM];
  vec4 split_start_distances;
  vec4 split_end_distances;
  vec4 shadow_vec_id;
};
BLI_STATIC_ASSERT_ALIGN(ShadowCascadeData, 16)

struct ShadowBlock {
  ShadowData BLOCK_ATTR(shadows_data)[MAX_SHADOW];
  ShadowCubeData BLOCK_ATTR(shadows_cube_data)[MAX_SHADOW_CUBE];
  ShadowCascadeData BLOCK_ATTR(shadows_cascade_data)[MAX_SHADOW_CASCADE];
};
BLI_STATIC_ASSERT_ALIGN(ShadowBlock, 16)

struct LightData {
  vec4 position_influence;     /* w : InfluenceRadius (inversed and squared) */
  vec4 color_influence_volume; /* w : InfluenceRadius but for Volume power */
  vec4 spotdata_radius_shadow; /* x : spot size, y : spot blend, z : radius, w: shadow id */
  vec4 rightvec_sizex;         /* xyz: Normalized up vector, w: area size X or spot scale X */
  vec4 upvec_sizey;            /* xyz: Normalized right vector, w: area size Y or spot scale Y */
  vec4 forwardvec_type;        /* xyz: Normalized forward vector, w: Light Type */
  vec4 diff_spec_volume;       /* xyz: Diffuse/Spec/Volume power, w: radius for volumetric. */
};
BLI_STATIC_ASSERT_ALIGN(LightData, 16)

struct LightBlock {
  LightData BLOCK_ATTR(lights_data)[MAX_LIGHT];
};
BLI_STATIC_ASSERT_ALIGN(LightBlock, 16)

struct RenderpassBlock {
  bool BLOCK_ATTR(renderPassDiffuse);
  bool BLOCK_ATTR(renderPassDiffuseLight);
  bool BLOCK_ATTR(renderPassGlossy);
  bool BLOCK_ATTR(renderPassGlossyLight);
  bool BLOCK_ATTR(renderPassEmit);
  bool BLOCK_ATTR(renderPassSSSColor);
  bool BLOCK_ATTR(renderPassEnvironment);
  bool BLOCK_ATTR(renderPassAOV);
  uint BLOCK_ATTR(renderPassAOVActive);
};
BLI_STATIC_ASSERT_ALIGN(RenderpassBlock, 16)

#define MAX_SSS_SAMPLES 65
#define SSS_LUT_SIZE 64.0
#define SSS_LUT_SCALE ((SSS_LUT_SIZE - 1.0) / float(SSS_LUT_SIZE))
#define SSS_LUT_BIAS (0.5 / float(SSS_LUT_SIZE))

struct SSSProfileBlock {
  vec4 _sss_kernel[MAX_SSS_SAMPLES];
  vec4 _radii_max_radius;
  float _avg_inv_radius;
  int _sss_samples;
};
BLI_STATIC_ASSERT_ALIGN(SSSProfileBlock, 16)

#ifdef GPU_SHADER

#  if defined(USE_GPU_SHADER_CREATE_INFO)

/* Keep compatibility_with old global scope syntax. */
#    define pastViewProjectionMatrix common_block.BLOCK_ATTR(pastViewProjectionMatrix)
#    define hizUvScale common_block.BLOCK_ATTR(hizUvScale)
#    define aoParameters common_block.BLOCK_ATTR(aoParameters)
#    define volTexSize common_block.BLOCK_ATTR(volTexSize)
#    define volDepthParameters common_block.BLOCK_ATTR(volDepthParameters)
#    define volInvTexSize common_block.BLOCK_ATTR(volInvTexSize)
#    define volJitter common_block.BLOCK_ATTR(volJitter)
#    define volCoordScale common_block.BLOCK_ATTR(volCoordScale)
#    define volHistoryAlpha common_block.BLOCK_ATTR(volHistoryAlpha)
#    define volShadowSteps common_block.BLOCK_ATTR(volShadowSteps)
#    define volUseLights common_block.BLOCK_ATTR(volUseLights)
#    define volUseSoftShadows common_block.BLOCK_ATTR(volUseSoftShadows)
#    define ssrParameters common_block.BLOCK_ATTR(ssrParameters)
#    define ssrBorderFac common_block.BLOCK_ATTR(ssrBorderFac)
#    define ssrMaxRoughness common_block.BLOCK_ATTR(ssrMaxRoughness)
#    define ssrFireflyFac common_block.BLOCK_ATTR(ssrFireflyFac)
#    define ssrBrdfBias common_block.BLOCK_ATTR(ssrBrdfBias)
#    define ssrToggle common_block.BLOCK_ATTR(ssrToggle)
#    define ssrefractToggle common_block.BLOCK_ATTR(ssrefractToggle)
#    define sssJitterThreshold common_block.BLOCK_ATTR(sssJitterThreshold)
#    define sssToggle common_block.BLOCK_ATTR(sssToggle)
#    define specToggle common_block.BLOCK_ATTR(specToggle)
#    define laNumLight common_block.BLOCK_ATTR(laNumLight)
#    define prbNumPlanar common_block.BLOCK_ATTR(prbNumPlanar)
#    define prbNumRenderCube common_block.BLOCK_ATTR(prbNumRenderCube)
#    define prbNumRenderGrid common_block.BLOCK_ATTR(prbNumRenderGrid)
#    define prbIrradianceVisSize common_block.BLOCK_ATTR(prbIrradianceVisSize)
#    define prbIrradianceSmooth common_block.BLOCK_ATTR(prbIrradianceSmooth)
#    define prbLodCubeMax common_block.BLOCK_ATTR(prbLodCubeMax)
#    define rayType common_block.BLOCK_ATTR(rayType)
#    define rayDepth common_block.BLOCK_ATTR(rayDepth)
#    define alphaHashOffset common_block.BLOCK_ATTR(alphaHashOffset)
#    define alphaHashScale common_block.BLOCK_ATTR(alphaHashScale)
#    define cameraUvScaleBias common_block.BLOCK_ATTR(cameraUvScaleBias)
#    define planarClipPlane common_block.BLOCK_ATTR(planarClipPlane)

/* ProbeBlock */
#    define probes_data probe_block.BLOCK_ATTR(probes_data)

/* GridBlock */
#    define grids_data grid_block.BLOCK_ATTR(grids_data)

/* PlanarBlock */
#    define planars_data planar_block.BLOCK_ATTR(planars_data)

/* ShadowBlock */
#    define shadows_data shadow_block.BLOCK_ATTR(shadows_data)
#    define shadows_cube_data shadow_block.BLOCK_ATTR(shadows_cube_data)
#    define shadows_cascade_data shadow_block.BLOCK_ATTR(shadows_cascade_data)

/* LightBlock */
#    define lights_data light_block.BLOCK_ATTR(lights_data)

/* RenderpassBlock */
#    define renderPassDiffuse renderpass_block.BLOCK_ATTR(renderPassDiffuse)
#    define renderPassDiffuseLight renderpass_block.BLOCK_ATTR(renderPassDiffuseLight)
#    define renderPassGlossy renderpass_block.BLOCK_ATTR(renderPassGlossy)
#    define renderPassGlossyLight renderpass_block.BLOCK_ATTR(renderPassGlossyLight)
#    define renderPassEmit renderpass_block.BLOCK_ATTR(renderPassEmit)
#    define renderPassSSSColor renderpass_block.BLOCK_ATTR(renderPassSSSColor)
#    define renderPassEnvironment renderpass_block.BLOCK_ATTR(renderPassEnvironment)
#    define renderPassAOV renderpass_block.BLOCK_ATTR(renderPassAOV)
#    define renderPassAOVActive renderpass_block.BLOCK_ATTR(renderPassAOVActive)

/* SSSProfileBlock */
#    define sss_kernel sssProfile.BLOCK_ATTR(sss_kernel)
#    define radii_max_radius sssProfile.BLOCK_ATTR(radii_max_radius)
#    define avg_inv_radius sssProfile.BLOCK_ATTR(avg_inv_radius)
#    define sss_samples sssProfile.BLOCK_ATTR(sss_samples)

#  endif /* USE_GPU_SHADER_CREATE_INFO */

#endif
#endif
