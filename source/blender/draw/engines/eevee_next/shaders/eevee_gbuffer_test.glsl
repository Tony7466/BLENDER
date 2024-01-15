/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Directive for resetting the line numbering so the failing tests lines can be printed.
 * This conflict with the shader compiler error logging scheme.
 * Comment out for correct compilation error line. */
#line 9

#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_test_lib.glsl)

#define TEST(a, b) if (true)

GBufferData gbuffer_new()
{
  GBufferData data;
  data.diffuse.weight = 0.0;
  data.translucent.weight = 0.0;
  data.reflection.weight = 0.0;
  data.refraction.weight = 0.0;
  data.thickness = 0.2;
  data.object_id = 0xF220u;
  data.surface_N = normalize(vec3(0.1, 0.2, 0.3));
  return data;
}

void main()
{
  GBufferData data_in;
  GBufferReader data_out;
  samplerGBufferHeader header_tx = 0;
  samplerGBufferClosure closure_tx = 0;
  samplerGBufferNormal normal_tx = 0;

  TEST(eevee_gbuffer, ClosureDiffuse)
  {
    data_in = gbuffer_new();
    data_in.diffuse.type = CLOSURE_BSDF_DIFFUSE_ID;
    data_in.diffuse.weight = 1.0;
    data_in.diffuse.color = vec3(0.1, 0.2, 0.3);
    data_in.diffuse.N = normalize(vec3(0.2, 0.1, 0.3));

    g_data_packed = gbuffer_pack(data_in);
    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(g_data_packed.layer_data, 1);
    EXPECT_EQ(data_out.closure_count, 1);

    ClosureUndetermined out_diffuse = gbuffer_closure_get(data_out, 0);

    EXPECT_EQ(out_diffuse.type, CLOSURE_BSDF_DIFFUSE_ID);
    EXPECT_EQ(data_in.diffuse.type, CLOSURE_BSDF_DIFFUSE_ID);
    EXPECT_NEAR(data_in.diffuse.color, out_diffuse.color, 1e-5);
    EXPECT_NEAR(data_in.diffuse.N, out_diffuse.N, 1e-5);
  }

  TEST(eevee_gbuffer, ClosureSubsurface)
  {
    data_in = gbuffer_new();
    data_in.diffuse.type = CLOSURE_BSSRDF_BURLEY_ID;
    data_in.diffuse.weight = 1.0;
    data_in.diffuse.color = vec3(0.1, 0.2, 0.3);
    data_in.diffuse.data.rgb = vec3(0.2, 0.3, 0.4);
    data_in.diffuse.N = normalize(vec3(0.2, 0.1, 0.3));

    g_data_packed = gbuffer_pack(data_in);
    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(g_data_packed.layer_data, 2);
    EXPECT_EQ(data_out.closure_count, 1);

    ClosureUndetermined out_sss_burley = gbuffer_closure_get(data_out, 0);

    EXPECT_EQ(out_sss_burley.type, CLOSURE_BSSRDF_BURLEY_ID);
    EXPECT_EQ(data_in.diffuse.type, CLOSURE_BSSRDF_BURLEY_ID);
    EXPECT_NEAR(data_in.diffuse.color, out_sss_burley.color, 1e-5);
    EXPECT_NEAR(data_in.diffuse.N, out_sss_burley.N, 1e-5);
    EXPECT_NEAR(data_in.diffuse.data.rgb, to_closure_subsurface(out_sss_burley).sss_radius, 1e-5);
  }

  TEST(eevee_gbuffer, ClosureTranslucent)
  {
    data_in = gbuffer_new();
    data_in.translucent.type = CLOSURE_BSDF_TRANSLUCENT_ID;
    data_in.translucent.weight = 1.0;
    data_in.translucent.color = vec3(0.1, 0.2, 0.3);
    data_in.translucent.N = normalize(vec3(0.2, 0.1, 0.3));

    g_data_packed = gbuffer_pack(data_in);
    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(g_data_packed.layer_data, 1);
    EXPECT_EQ(data_out.closure_count, 1);

    ClosureUndetermined out_translucent = gbuffer_closure_get(data_out, 0);

    EXPECT_EQ(out_translucent.type, CLOSURE_BSDF_TRANSLUCENT_ID);
    EXPECT_EQ(data_in.translucent.type, CLOSURE_BSDF_TRANSLUCENT_ID);
    EXPECT_NEAR(data_in.translucent.color, out_translucent.color, 1e-5);
    EXPECT_NEAR(data_in.translucent.N, out_translucent.N, 1e-5);
  }

  TEST(eevee_gbuffer, ClosureReflection)
  {
    data_in = gbuffer_new();
    data_in.reflection.type = CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID;
    data_in.reflection.weight = 1.0;
    data_in.reflection.color = vec3(0.1, 0.2, 0.3);
    data_in.reflection.data.x = 0.4;
    data_in.reflection.N = normalize(vec3(0.2, 0.1, 0.3));

    g_data_packed = gbuffer_pack(data_in);
    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(g_data_packed.layer_data, 2);
    EXPECT_EQ(data_out.closure_count, 1);

    ClosureUndetermined out_reflection = gbuffer_closure_get(data_out, 0);

    EXPECT_EQ(out_reflection.type, CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);
    EXPECT_EQ(data_in.reflection.type, CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);
    EXPECT_NEAR(data_in.reflection.color, out_reflection.color, 1e-5);
    EXPECT_NEAR(data_in.reflection.N, out_reflection.N, 1e-5);
    EXPECT_NEAR(data_in.reflection.data.r, out_reflection.data.r, 1e-5);
  }

  TEST(eevee_gbuffer, ClosureRefraction)
  {
    data_in = gbuffer_new();
    data_in.refraction.type = CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID;
    data_in.refraction.weight = 1.0;
    data_in.refraction.color = vec3(0.1, 0.2, 0.3);
    data_in.refraction.data.x = 0.4;
    data_in.refraction.data.y = 0.5;
    data_in.refraction.N = normalize(vec3(0.2, 0.1, 0.3));

    g_data_packed = gbuffer_pack(data_in);
    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(g_data_packed.layer_data, 2);
    EXPECT_EQ(data_out.closure_count, 1);

    ClosureUndetermined out_refraction = gbuffer_closure_get(data_out, 0);

    EXPECT_EQ(out_refraction.type, CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID);
    EXPECT_EQ(data_in.refraction.type, CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID);
    EXPECT_NEAR(data_in.refraction.color, out_refraction.color, 1e-5);
    EXPECT_NEAR(data_in.refraction.N, out_refraction.N, 1e-5);
    EXPECT_NEAR(data_in.refraction.data.r, out_refraction.data.r, 1e-5);
    EXPECT_NEAR(data_in.refraction.data.g, out_refraction.data.g, 1e-5);
  }

  TEST(eevee_gbuffer, ClosureCombination)
  {
    ClosureUndetermined in_cl0 = closure_new(CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID);
    in_cl0.weight = 1.0;
    in_cl0.color = vec3(0.1, 0.2, 0.3);
    in_cl0.data.x = 0.4;
    in_cl0.data.y = 0.5;
    in_cl0.N = normalize(vec3(0.2, 0.1, 0.3));

    ClosureUndetermined in_cl1 = closure_new(CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);
    in_cl1.weight = 1.0;
    in_cl1.color = vec3(0.4, 0.5, 0.6);
    in_cl1.data.x = 0.6;
    in_cl1.N = normalize(vec3(0.2, 0.3, 0.4));

    data_in = gbuffer_new();
    data_in.refraction = in_cl0;
    data_in.reflection = in_cl1;

    g_data_packed = gbuffer_pack(data_in);

    EXPECT_EQ(g_data_packed.layer_data, 4);

    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(data_out.closure_count, 2);

    ClosureUndetermined out_cl0 = gbuffer_closure_get(data_out, 0);
    ClosureUndetermined out_cl1 = gbuffer_closure_get(data_out, 1);

    EXPECT_EQ(out_cl0.type, CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID);
    EXPECT_EQ(in_cl0.type, CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID);
    EXPECT_NEAR(in_cl0.color, out_cl0.color, 1e-5);
    EXPECT_NEAR(in_cl0.N, out_cl0.N, 1e-5);
    EXPECT_NEAR(in_cl0.data.r, out_cl0.data.r, 1e-5);
    EXPECT_NEAR(in_cl0.data.g, out_cl0.data.g, 1e-5);

    EXPECT_EQ(out_cl1.type, CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);
    EXPECT_EQ(in_cl1.type, CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);
    EXPECT_NEAR(in_cl1.color, out_cl1.color, 1e-5);
    EXPECT_NEAR(in_cl1.N, out_cl1.N, 1e-5);
    EXPECT_NEAR(in_cl1.data.r, out_cl1.data.r, 1e-5);
  }

  TEST(eevee_gbuffer, ClosureColorless)
  {
    data_in = gbuffer_new();
    data_in.refraction.type = CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID;
    data_in.refraction.weight = 1.0;
    data_in.refraction.color = vec3(0.1, 0.1, 0.1);
    data_in.refraction.data.x = 0.4;
    data_in.refraction.data.y = 0.5;
    data_in.refraction.N = normalize(vec3(0.2, 0.1, 0.3));

    data_in.reflection.type = CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID;
    data_in.reflection.weight = 1.0;
    data_in.reflection.color = vec3(0.1, 0.1, 0.1);
    data_in.reflection.data.x = 0.4;
    data_in.reflection.N = normalize(vec3(0.2, 0.3, 0.4));

    g_data_packed = gbuffer_pack(data_in);

    EXPECT_EQ(g_data_packed.layer_data, 2);

    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(data_out.closure_count, 2);

    ClosureUndetermined out_reflection = gbuffer_closure_get(data_out, 1);
    ClosureUndetermined out_refraction = gbuffer_closure_get(data_out, 0);

    EXPECT_EQ(out_refraction.type, CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID);
    EXPECT_EQ(data_in.refraction.type, CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID);
    EXPECT_NEAR(data_in.refraction.color, out_refraction.color, 1e-5);
    EXPECT_NEAR(data_in.refraction.N, out_refraction.N, 1e-5);
    EXPECT_NEAR(data_in.refraction.data.r, out_refraction.data.r, 1e-5);
    EXPECT_NEAR(data_in.refraction.data.g, out_refraction.data.g, 1e-5);

    EXPECT_EQ(out_reflection.type, CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);
    EXPECT_EQ(data_in.reflection.type, CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);
    EXPECT_NEAR(data_in.reflection.color, out_reflection.color, 1e-5);
    EXPECT_NEAR(data_in.reflection.N, out_reflection.N, 1e-5);
    EXPECT_NEAR(data_in.reflection.data.r, out_reflection.data.r, 1e-5);
  }

  TEST(eevee_gbuffer, NormalPack)
  {
    GBufferWriter gbuf;
    gbuf.header = 0u;
    gbuf.layer_gbuf = 0;
    gbuf.layer_data = 0;
    gbuf.layer_normal = 0;

    vec3 N0 = normalize(vec3(0.2, 0.1, 0.3));
    vec3 N1 = normalize(vec3(0.1, 0.2, 0.3));
    vec3 N2 = normalize(vec3(0.3, 0.1, 0.2));

    gbuffer_append_closure(gbuf, GBUF_DIFFUSE);
    gbuffer_append_normal(gbuf, N0);

    EXPECT_EQ(gbuf.layer_gbuf, 1);
    EXPECT_EQ(gbuf.layer_normal, 1);
    EXPECT_EQ(gbuf.N[0], gbuffer_normal_pack(N0));

    gbuffer_append_closure(gbuf, GBUF_DIFFUSE);
    gbuffer_append_normal(gbuf, N1);

    EXPECT_EQ(gbuf.layer_gbuf, 2);
    EXPECT_EQ(gbuf.layer_normal, 2);
    EXPECT_EQ(gbuf.N[1], gbuffer_normal_pack(N1));

    gbuffer_append_closure(gbuf, GBUF_DIFFUSE);
    gbuffer_append_normal(gbuf, N2);

    EXPECT_EQ(gbuf.layer_gbuf, 3);
    EXPECT_EQ(gbuf.layer_normal, 3);
    EXPECT_EQ(gbuf.N[2], gbuffer_normal_pack(N2));
  }

  TEST(eevee_gbuffer, NormalPackOpti)
  {
    GBufferWriter gbuf;
    gbuf.header = 0u;
    gbuf.layer_gbuf = 0;
    gbuf.layer_data = 0;
    gbuf.layer_normal = 0;

    vec3 N0 = normalize(vec3(0.2, 0.1, 0.3));

    gbuffer_append_closure(gbuf, GBUF_DIFFUSE);
    gbuffer_append_normal(gbuf, N0);

    EXPECT_EQ(gbuf.layer_gbuf, 1);
    EXPECT_EQ(gbuf.layer_normal, 1);
    EXPECT_EQ(gbuf.N[0], gbuffer_normal_pack(N0));

    gbuffer_append_closure(gbuf, GBUF_DIFFUSE);
    gbuffer_append_normal(gbuf, N0);

    EXPECT_EQ(gbuf.layer_gbuf, 2);
    EXPECT_EQ(gbuf.layer_normal, 1);

    gbuffer_append_closure(gbuf, GBUF_DIFFUSE);
    gbuffer_append_normal(gbuf, N0);

    EXPECT_EQ(gbuf.layer_gbuf, 3);
    EXPECT_EQ(gbuf.layer_normal, 1);
  }

  TEST(eevee_gbuffer, NormalReuse)
  {
    ClosureUndetermined cl1 = closure_new(CLOSURE_BSDF_DIFFUSE_ID);
    cl1.weight = 1.0;
    cl1.color = vec3(1);
    cl1.N = normalize(vec3(0.2, 0.1, 0.3));

    ClosureUndetermined cl2 = closure_new(CLOSURE_BSDF_DIFFUSE_ID);
    cl2.weight = 1.0;
    cl2.color = vec3(1);
    cl2.N = normalize(vec3(0.1, 0.2, 0.3));

    ClosureUndetermined cl3 = closure_new(CLOSURE_BSDF_DIFFUSE_ID);
    cl3.weight = 1.0;
    cl3.color = vec3(1);
    cl3.N = normalize(vec3(0.3, 0.2, 0.1));

    /* Double closure reuse. */
    data_in = gbuffer_new();
    data_in.refraction = cl1;
    data_in.reflection = cl1;

    g_data_packed = gbuffer_pack(data_in);

    EXPECT_EQ(g_data_packed.layer_gbuf, 2);
    EXPECT_EQ(g_data_packed.layer_normal, 1);

    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(data_out.closure_count, 2);
    EXPECT_EQ(data_out.layer_normal, 1);
    EXPECT_NEAR(cl1.N, gbuffer_closure_get(data_out, 0).N, 1e-5);
    EXPECT_NEAR(cl1.N, gbuffer_closure_get(data_out, 1).N, 1e-5);

    /* Double closure no reuse. */
    data_in = gbuffer_new();
    data_in.refraction = cl1;
    data_in.reflection = cl2;

    g_data_packed = gbuffer_pack(data_in);

    EXPECT_EQ(g_data_packed.layer_gbuf, 2);
    EXPECT_EQ(g_data_packed.layer_normal, 2);

    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(data_out.closure_count, 2);
    EXPECT_EQ(data_out.layer_normal, 2);
    EXPECT_NEAR(cl1.N, gbuffer_closure_get(data_out, 0).N, 1e-5);
    EXPECT_NEAR(cl2.N, gbuffer_closure_get(data_out, 1).N, 1e-5);

    /* Triple closure reuse 1st. */
    data_in = gbuffer_new();
    data_in.diffuse = cl1;
    data_in.refraction = cl2;
    data_in.reflection = cl2;

    g_data_packed = gbuffer_pack(data_in);

    EXPECT_EQ(g_data_packed.layer_normal, 2);

    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(data_out.closure_count, 3);
    EXPECT_EQ(data_out.layer_normal, 2);
    EXPECT_NEAR(cl1.N, gbuffer_closure_get(data_out, 0).N, 1e-5);
    EXPECT_NEAR(cl2.N, gbuffer_closure_get(data_out, 1).N, 1e-5);
    EXPECT_NEAR(cl2.N, gbuffer_closure_get(data_out, 2).N, 1e-5);

    /* Triple closure reuse 2nd. */
    data_in = gbuffer_new();
    data_in.diffuse = cl2;
    data_in.refraction = cl1;
    data_in.reflection = cl2;

    g_data_packed = gbuffer_pack(data_in);

    EXPECT_EQ(g_data_packed.layer_normal, 2);

    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(data_out.closure_count, 3);
    EXPECT_EQ(data_out.layer_normal, 2);
    EXPECT_NEAR(cl2.N, gbuffer_closure_get(data_out, 0).N, 1e-5);
    EXPECT_NEAR(cl1.N, gbuffer_closure_get(data_out, 1).N, 1e-5);
    EXPECT_NEAR(cl2.N, gbuffer_closure_get(data_out, 2).N, 1e-5);

    /* Triple closure reuse 3rd. */
    data_in = gbuffer_new();
    data_in.diffuse = cl2;
    data_in.refraction = cl2;
    data_in.reflection = cl1;

    g_data_packed = gbuffer_pack(data_in);

    EXPECT_EQ(g_data_packed.layer_normal, 2);

    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(data_out.closure_count, 3);
    EXPECT_EQ(data_out.layer_normal, 2);
    EXPECT_NEAR(cl2.N, gbuffer_closure_get(data_out, 0).N, 1e-5);
    EXPECT_NEAR(cl2.N, gbuffer_closure_get(data_out, 1).N, 1e-5);
    EXPECT_NEAR(cl1.N, gbuffer_closure_get(data_out, 2).N, 1e-5);

    /* Triple closure reuse none. */
    data_in = gbuffer_new();
    data_in.diffuse = cl1;
    data_in.refraction = cl2;
    data_in.reflection = cl3;

    g_data_packed = gbuffer_pack(data_in);

    EXPECT_EQ(g_data_packed.layer_normal, 3);

    data_out = gbuffer_read(header_tx, closure_tx, normal_tx, ivec2(0));

    EXPECT_EQ(data_out.closure_count, 3);
    EXPECT_EQ(data_out.layer_normal, 3);
    EXPECT_NEAR(cl1.N, gbuffer_closure_get(data_out, 0).N, 1e-5);
    EXPECT_NEAR(cl2.N, gbuffer_closure_get(data_out, 1).N, 1e-5);
    EXPECT_NEAR(cl3.N, gbuffer_closure_get(data_out, 2).N, 1e-5);
  }
}
