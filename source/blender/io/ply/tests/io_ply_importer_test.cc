/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "testing/testing.h"

#include "BLI_fileops.hh"
#include "ply_import.hh"
#include "ply_import_ascii.hh"
#include "ply_import_binary.hh"

namespace blender::io::ply {

struct Expectation {
  int totvert, totpoly, totindex, totedge;
  float3 vert_first, vert_last;
  float3 normal_first = {0, 0, 0};
  float2 uv_first;
  float4 color_first = {-1, -1, -1, -1};
};

class ply_import_test : public testing::Test {
 public:
  void import_and_check(const char *path, const Expectation &exp)
  {
    std::string ply_path = blender::tests::flags_test_asset_dir() + "/io_tests/ply/" + path;

    fstream infile(ply_path, std::ios::in | std::ios::binary);
    PlyHeader header;
    const char *header_err = read_header(infile, header);
    if (header_err != nullptr) {
      ADD_FAILURE();
      return;
    }
    std::unique_ptr<PlyData> data;
    try {
      if (header.type == PlyFormatType::ASCII) {
        data = import_ply_ascii(infile, header);
      }
      else {
        data = import_ply_binary(infile, &header);
      }
    }
    catch (std::exception &e) {
      ASSERT_EQ(0, exp.totvert);
      ASSERT_EQ(0, exp.totpoly);
      return;
    }

    /* Test expected amount of vertices, edges, and faces. */
    ASSERT_EQ(header.vertex_count, exp.totvert);
    ASSERT_EQ(data->vertices.size(), exp.totvert);
    ASSERT_EQ(header.edge_count, exp.totedge);
    ASSERT_EQ(data->edges.size(), exp.totedge);
    ASSERT_EQ(header.face_count, exp.totpoly);
    ASSERT_EQ(data->faces.size(), exp.totpoly);

    int indexCount = 0;
    for (const auto &f : data->faces) {
      indexCount += f.size();
    }
    ASSERT_EQ(indexCount, exp.totindex);

    /* Test if first and last vertices match. */
    EXPECT_V3_NEAR(data->vertices.first(), exp.vert_first, 0.0001f);
    EXPECT_V3_NEAR(data->vertices.last(), exp.vert_last, 0.0001f);

    /* Check if first normal matches. */
    float3 got_normal = data->vertex_normals.is_empty() ? float3(0, 0, 0) :
                                                          data->vertex_normals.first();
    EXPECT_V3_NEAR(got_normal, exp.normal_first, 0.0001f);

    /* Check if first UV matches. */
    float2 got_uv = data->uv_coordinates.is_empty() ? float2(0, 0) : data->uv_coordinates.first();
    EXPECT_V2_NEAR(got_uv, exp.uv_first, 0.0001f);

    /* Check if first color matches. */
    float4 got_color = data->vertex_colors.is_empty() ? float4(-1, -1, -1, -1) :
                                                        data->vertex_colors.first();
    EXPECT_V4_NEAR(got_color, exp.color_first, 0.0001f);
  }
};

TEST_F(ply_import_test, PLYImportCube)
{
  Expectation expect = {24,
                        6,
                        24,
                        0,
                        float3(1, 1, -1),
                        float3(-1, 1, 1),
                        float3(0, 0, -1),
                        float2(0.979336, 0.844958),
                        float4(1, 0.8470, 0, 1)};
  import_and_check("cube_ascii.ply", expect);
}

TEST_F(ply_import_test, PLYImportASCIIEdgeTest)
{
  Expectation expect = {8, 0, 0, 12, float3(-1, -1, -1), float3(1, 1, 1)};
  import_and_check("ASCII_wireframe_cube.ply", expect);
}

TEST_F(ply_import_test, PLYImportBunny)
{
  Expectation expect = {1623,
                        1000,
                        3000,
                        0,
                        float3(0.0380425, 0.109755, 0.0161689),
                        float3(-0.0722821, 0.143895, -0.0129091)};
  import_and_check("bunny2.ply", expect);
}

TEST_F(ply_import_test, PlyImportManySmallHoles)
{
  Expectation expect = {2004,
                        3524,
                        10572,
                        0,
                        float3(-0.0131592, -0.0598382, 1.58958),
                        float3(-0.0177622, 0.0105153, 1.61977),
                        float3(0, 0, 0),
                        float2(0, 0),
                        float4(0.7215, 0.6784, 0.6627, 1)};
  import_and_check("many_small_holes.ply", expect);
}

TEST_F(ply_import_test, PlyImportWireframeCube)
{
  Expectation expect = {8, 0, 0, 12, float3(-1, -1, -1), float3(1, 1, 1)};
  import_and_check("wireframe_cube.ply", expect);
}

TEST_F(ply_import_test, PlyImportColorNotFull)
{
  Expectation expect = {4, 1, 4, 0, float3(1, 0, 1), float3(-1, 0, 1)};
  import_and_check("color_not_full_a.ply", expect);
  // import_and_check("color_not_full_b.ply", expect);
}

TEST_F(ply_import_test, PlyImportDoubleXYZ)
{
  Expectation expect = {4,
                        1,
                        4,
                        0,
                        float3(1, 0, 1),
                        float3(-1, 0, 1),
                        float3(0, 0, 0),
                        float2(0, 0),
                        float4(1, 0, 0, 1)};
  import_and_check("double_xyz_a.ply", expect);
  // import_and_check("double_xyz_b.ply", expect);
}

TEST_F(ply_import_test, PlyImportFaceUVsColors)
{
  Expectation expect = {4, 1, 4, 0, float3(1, 0, 1), float3(-1, 0, 1)};
  import_and_check("face_uvs_colors_a.ply", expect);
  // import_and_check("face_uvs_colors_b.ply", expect);
}

TEST_F(ply_import_test, PlyImportFacesFirst)
{
  Expectation expect = {4,
                        1,
                        4,
                        0,
                        float3(1, 0, 1),
                        float3(-1, 0, 1),
                        float3(0, 0, 0),
                        float2(0, 0),
                        float4(1, 0, 0, 1)};
  import_and_check("faces_first_a.ply", expect);
  // import_and_check("faces_first_b.ply", expect);
}

TEST_F(ply_import_test, PlyImportFloatFormats)
{
  Expectation expect = {4,
                        1,
                        4,
                        0,
                        float3(1, 0, 1),
                        float3(-1, 0, 1),
                        float3(0, 0, 0),
                        float2(0, 0),
                        float4(0.5f, 0, 0.25f, 1)};
  import_and_check("float_formats_a.ply", expect);
  // import_and_check("float_formats_b.ply", expect);
}

TEST_F(ply_import_test, PlyImportPositionNotFull)
{
  Expectation expect = {0, 0, 0, 0};
  import_and_check("position_not_full_a.ply", expect);
  // import_and_check("position_not_full_b.ply", expect);
}

TEST_F(ply_import_test, PlyImportTristrips)
{
  Expectation expect = {6, 0, 0, 0, float3(1, 0, 1), float3(-3, 0, 1)};  //@TODO: incorrect
  import_and_check("tristrips_a.ply", expect);
  // import_and_check("tristrips_b.ply", expect);
}

TEST_F(ply_import_test, PlyImportTypeAliases)
{
  Expectation expect = {4,
                        1,
                        4,
                        0,
                        float3(1, 0, 1),
                        float3(-1, 0, 1),
                        float3(0, 0, 0),
                        float2(0, 0),
                        float4(220 / 255.0f, 20 / 255.0f, 20 / 255.0f, 1)};
  import_and_check("type_aliases_a.ply", expect);
  // import_and_check("type_aliases_b.ply", expect);
}

TEST_F(ply_import_test, PlyImportVertexCompOrder)
{
  Expectation expect = {4,
                        1,
                        4,
                        0,
                        float3(1, 0, 1),
                        float3(-1, 0, 1),
                        float3(0, 0, 0),
                        float2(0, 0),
                        float4(0.8f, 0.2f, 0, 1)};
  import_and_check("vertex_comp_order_a.ply", expect);
  // import_and_check("vertex_comp_order_b.ply", expect);
}

TEST(ply_import_functions_test, PlySwapBytes)
{
  /* Individual bits shouldn't swap with each other. */
  uint8_t val8 = 0xA8;
  uint8_t exp8 = 0xA8;
  uint8_t actual8 = swap_bytes<uint8_t>(val8);
  ASSERT_EQ(exp8, actual8);

  uint16_t val16 = 0xFEB0;
  uint16_t exp16 = 0xB0FE;
  uint16_t actual16 = swap_bytes<uint16_t>(val16);
  ASSERT_EQ(exp16, actual16);

  uint32_t val32 = 0x80A37B0A;
  uint32_t exp32 = 0x0A7BA380;
  uint32_t actual32 = swap_bytes<uint32_t>(val32);
  ASSERT_EQ(exp32, actual32);

  uint64_t val64 = 0x0102030405060708;
  uint64_t exp64 = 0x0807060504030201;
  uint64_t actual64 = swap_bytes<uint64_t>(val64);
  ASSERT_EQ(exp64, actual64);
}

}  // namespace blender::io::ply
