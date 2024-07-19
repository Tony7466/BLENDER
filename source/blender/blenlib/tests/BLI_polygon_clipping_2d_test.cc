/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "MEM_guardedalloc.h"

#include "BLI_rand.h"
#include "BLI_time.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <type_traits>

/* Should tests draw their output to an HTML file? */
#define DO_DRAW 1

#include "BLI_array.hh"
#include "BLI_vector.hh"

#include "BLI_polygon_clipping_2d.hh"

namespace blender::polygonboolean {

static bool draw_append = false; /* Will be set to true after first call. */

void draw_curve(const std::string &label,
                const Span<float2> curve_a,
                const Span<float2> curve_b,
                const BooleanResult &result)
{
  /* Would like to use BKE_tempdir_base() here, but that brings in dependence on kernel library.
   * This is just for developer debugging anyway, and should never be called in production Blender.
   */
#ifdef WIN32
  constexpr const char *drawfile = "./polygon_clipping_test_draw.html";
#else
  constexpr const char *drawfile = "/tmp/polygon_clipping_test_draw.html";
#endif
  constexpr int max_draw_width = 800;
  constexpr int max_draw_height = 600;
  constexpr int stroke_width = 3;

  float2 vmin(1e10, 1e10);
  float2 vmax(-1e10, -1e10);
  for (const float2 &v : curve_a) {
    for (int i = 0; i < 2; ++i) {
      float vi = v[i];
      if (vi < vmin[i]) {
        vmin[i] = vi;
      }
      if (vi > vmax[i]) {
        vmax[i] = vi;
      }
    }
  }
  for (const float2 &v : curve_b) {
    for (int i = 0; i < 2; ++i) {
      float vi = v[i];
      if (vi < vmin[i]) {
        vmin[i] = vi;
      }
      if (vi > vmax[i]) {
        vmax[i] = vi;
      }
    }
  }
  float draw_margin = ((vmax.x - vmin.x) + (vmax.y - vmin.y)) * 0.05;
  float minx = vmin.x - draw_margin;
  float maxx = vmax.x + draw_margin;
  float miny = vmin.y - draw_margin;
  float maxy = vmax.y + draw_margin;

  float width = maxx - minx;
  float height = maxy - miny;
  float aspect = height / width;
  int view_width = max_draw_width;
  int view_height = int(view_width * aspect);
  if (view_height > max_draw_height) {
    view_height = max_draw_height;
    view_width = int(view_height / aspect);
  }
  float scale = view_width / width;

#define SX(x) ((x - minx) * scale)
#define SY(y) ((maxy - y) * scale)

  std::ofstream f;
  if (draw_append) {
    f.open(drawfile, std::ios_base::app);
  }
  else {
    f.open(drawfile);
  }
  if (!f) {
    std::cout << "Could not open file " << drawfile << "\n";
    return;
  }

  if (!draw_append) {
    f << "<!DOCTYPE html>\n";
  }

  f << "<div style=\"border: 5px solid black; text-align: center;\" >\n";
  f << "<h1>" << label << "</h1>\n";

  f << "<svg version=\"1.1\" "
       "xmlns=\"http://www.w3.org/2000/svg\" "
       "xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
       "xml:space=\"preserve\"\n"
    << "width=\"" << view_width << "\" height=\"" << view_height << "\">\n";

  f << "<polygon fill = \"green\" stroke =\"none\" "
       "points =\"";
  for (const int i : result.verts.index_range()) {
    const Vertex &vert = result.verts[i];
    const VertexType &type = vert.type;

    float2 point;
    if (type == VertexType::PointA) {
      point = curve_a[vert.point_id];
    }
    else if (type == VertexType::PointB) {
      point = curve_b[vert.point_id];
    }
    else if (type == VertexType::Intersection) {
      const IntersectionPoint &inter_point = result.intersections_data[vert.point_id];

      const float2 point_a0 = curve_a[inter_point.point_a];
      const float2 point_a1 = curve_a[(inter_point.point_a + 1) % curve_a.size()];
      const float alpha_a = inter_point.alpha_a;

      point = (1.0 - alpha_a) * point_a0 + alpha_a * point_a1;
    }

    f << SX(point[0]) << "," << SY(point[1]) << ", ";
  }
  f << "\"/>\n";

  f << "<polygon fill = \"none\" stroke =\"red\" "
       "stroke-width=\""
    << stroke_width
    << "\" "
       "points =\"";
  for (const float2 &point : curve_a) {
    f << SX(point[0]) << "," << SY(point[1]) << ", ";
  }
  f << "\"/>\n";

  f << "<polygon fill = \"none\" stroke =\"blue\" "
       "stroke-width=\""
    << stroke_width
    << "\" "
       "points =\"";
  for (const float2 &point : curve_b) {
    f << SX(point[0]) << "," << SY(point[1]) << ", ";
  }
  f << "\"/>\n";

  f << "</div>\n";

  draw_append = true;
#undef SX
#undef SY
}

void squares_A_AND_B_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  const BooleanMode mode = BooleanMode::A_AND_B;
  BooleanResult result = polygonboolean::curve_boolean_calc(mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 4);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Squares A*B", points_a, points_b, result);
  }
}

void squares_A_OR_B_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  const BooleanMode mode = BooleanMode::A_OR_B;
  BooleanResult result = polygonboolean::curve_boolean_calc(mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 8);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Squares A+B", points_a, points_b, result);
  }
}

void squares_A_NOT_B_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  const BooleanMode mode = BooleanMode::A_NOT_B;
  BooleanResult result = polygonboolean::curve_boolean_calc(mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 6);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Squares A-B", points_a, points_b, result);
  }
}

void squares_B_NOT_A_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  const BooleanMode mode = BooleanMode::B_NOT_A;
  BooleanResult result = polygonboolean::curve_boolean_calc(mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 6);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Squares B-A", points_a, points_b, result);
  }
}

TEST(polygonboolean, Squares_A_AND_B)
{
  squares_A_AND_B_test();
}

TEST(polygonboolean, Squares_A_OR_B)
{
  squares_A_OR_B_test();
}

TEST(polygonboolean, Squares_A_NOT_B)
{
  squares_A_NOT_B_test();
}

TEST(polygonboolean, Squares_B_NOT_A)
{
  squares_B_NOT_A_test();
}

}  // namespace blender::polygonboolean
