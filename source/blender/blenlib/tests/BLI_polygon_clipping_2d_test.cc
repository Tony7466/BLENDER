/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "MEM_guardedalloc.h"

#include "BLI_offset_indices.hh"
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
  constexpr int border_width = 5;
  constexpr int stroke_width = 3;
  constexpr int stroke_dasharray = 15;

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

    f << "<style>\n";

    f << "div {\n"
         "  border: "
      << border_width
      << "px solid black;\n"
         "  text-align: center;\n"
         "}\n";

    f << ".polygon-A {\n"
         "  fill: none;\n"
         "  stroke: red;\n"
         "  stroke-width: "
      << stroke_width
      << "px;\n"
         "  stroke-dasharray: "
      << stroke_dasharray
      << "px;\n"
         "}\n";

    f << ".polygon-B {\n"
         "  fill: none;\n"
         "  stroke: blue;\n"
         "  stroke-width: "
      << stroke_width
      << "px;\n"
         "  stroke-dasharray: "
      << stroke_dasharray
      << "px;\n"
         "}\n";

    f << ".polygon-C {\n"
         "  fill: green;\n"
         "  stroke: black;\n"
         "  stroke-width: "
      << stroke_width + 1
      << "px;\n"
         "  fill-opacity: 0.75;\n"
         "}\n";

    f << "</style>\n";
  }

  f << "<div>\n";
  f << "<h1>" << label << "</h1>\n";

  f << "<svg width=\"" << view_width << "\" height=\"" << view_height << "\">\n";

  f << "<polygon class = \"polygon-A\" "
       "points =\"";
  for (const int i : curve_a.index_range()) {
    const float2 &point = curve_a[i];
    if (i != 0) {
      f << ", ";
    }
    f << SX(point[0]) << "," << SY(point[1]);
  }
  f << "\"/>\n";

  f << "<polygon class = \"polygon-B\" "
       "points =\"";
  for (const int i : curve_b.index_range()) {
    const float2 &point = curve_b[i];
    if (i != 0) {
      f << ", ";
    }
    f << SX(point[0]) << "," << SY(point[1]);
  }
  f << "\"/>\n";

  const OffsetIndices<int> points_by_polygon = OffsetIndices<int>(result.offsets);

  f << "<path class = \"polygon-C\" d = \"";
  for (const int polygon_id : points_by_polygon.index_range()) {
    const IndexRange vert_ids = points_by_polygon[polygon_id];
    const Span<Vertex> verts = result.verts.as_span().slice(vert_ids);
    if (polygon_id != 0) {
      f << " ";
    }

    f << "M ";
    for (const int i : verts.index_range()) {
      const Vertex &vert = verts[i];
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

      if (i == 1) {
        f << " L ";
      }
      else if (i != 0) {
        f << ", ";
      }
      f << SX(point[0]) << "," << SY(point[1]);
    }
    f << " Z";
  }

  f << "\"";

  if (points_by_polygon.size() > 1) {
    f << " fill-rule=\"evenodd\"";
  }

  f << "/>\n";

  f << "</div>\n";

  draw_append = true;
#undef SX
#undef SY
}

void squares_A_AND_B_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_AND_B;
  input_mode.hole_mode = HoleMode::WITH_HOLES;

  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 4);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Squares A intersection B", points_a, points_b, result);
  }
}

void squares_A_OR_B_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_OR_B;
  input_mode.hole_mode = HoleMode::WITH_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 8);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Squares A union B", points_a, points_b, result);
  }
}

void squares_A_NOT_B_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_NOT_B;
  input_mode.hole_mode = HoleMode::WITH_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 6);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Squares A without B", points_a, points_b, result);
  }
}

void squares_B_NOT_A_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::B_NOT_A;
  input_mode.hole_mode = HoleMode::WITH_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 6);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Squares B without A", points_a, points_b, result);
  }
}

void simple_intersection_test()
{
  /**
   * This is a replica of Fig. 10 from
   * Greiner, Günther; Kai Hormann (1998). "Efficient clipping of arbitrary polygons". ACM
   * Transactions on Graphics. 17 (2): 71–83.
   */
  const Array<float2> points_a = {{0, 6}, {8, 6}, {8, 3}, {0, 3}};
  const Array<float2> points_b = {{6, 0}, {6, 4}, {4, 2}, {2, 4}, {2, 0}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_AND_B;
  input_mode.hole_mode = HoleMode::WITH_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 6);
  EXPECT_EQ(result.intersections_data.size(), 4);
  EXPECT_EQ(result.offsets.size(), 3);

  if (DO_DRAW) {
    draw_curve("Simple Intersection", points_a, points_b, result);
  }
}

void simple_union_with_hole_test()
{
  /**
   * This is a replica of Fig. 10 from
   * Greiner, Günther; Kai Hormann (1998). "Efficient clipping of arbitrary polygons". ACM
   * Transactions on Graphics. 17 (2): 71–83.
   */
  const Array<float2> points_a = {{0, 6}, {8, 6}, {8, 3}, {0, 3}};
  const Array<float2> points_b = {{6, 0}, {6, 4}, {4, 2}, {2, 4}, {2, 0}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_OR_B;
  input_mode.hole_mode = HoleMode::WITH_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 11);
  EXPECT_EQ(result.intersections_data.size(), 4);
  EXPECT_EQ(result.offsets.size(), 3);

  if (DO_DRAW) {
    draw_curve("Simple Union With Hole", points_a, points_b, result);
  }
}

void simple_union_without_hole_test()
{
  /**
   * This is a replica of Fig. 10 from
   * Greiner, Günther; Kai Hormann (1998). "Efficient clipping of arbitrary polygons". ACM
   * Transactions on Graphics. 17 (2): 71–83.
   */
  const Array<float2> points_a = {{0, 6}, {8, 6}, {8, 3}, {0, 3}};
  const Array<float2> points_b = {{6, 0}, {6, 4}, {4, 2}, {2, 4}, {2, 0}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_OR_B;
  input_mode.hole_mode = HoleMode::WITHOUT_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 8);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Simple Union Without Hole", points_a, points_b, result);
  }
}

void complex_A_AND_B_test()
{
  /**
   * This is a replica of Fig. 16 from
   * Greiner, Günther; Kai Hormann (1998). "Efficient clipping of arbitrary polygons". ACM
   * Transactions on Graphics. 17 (2): 71–83.
   */
  const Array<float2> points_a = {{14, 1}, {0, 5}, {14, 10}, {5, 6}, {14, 6}, {5, 5}};
  const Array<float2> points_b = {{9, 13}, {13, 0}, {9, 9}, {6, 0}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_AND_B;
  input_mode.hole_mode = HoleMode::WITH_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 24);
  EXPECT_EQ(result.intersections_data.size(), 24);
  EXPECT_EQ(result.offsets.size(), 7);

  if (DO_DRAW) {
    draw_curve("Complex A Intersection B", points_a, points_b, result);
  }
}

void complex_A_OR_B_test()
{
  /**
   * This is a replica of Fig. 16 from
   * Greiner, Günther; Kai Hormann (1998). "Efficient clipping of arbitrary polygons". ACM
   * Transactions on Graphics. 17 (2): 71–83.
   */
  const Array<float2> points_a = {{14, 1}, {0, 5}, {14, 10}, {5, 6}, {14, 6}, {5, 5}};
  const Array<float2> points_b = {{9, 13}, {13, 0}, {9, 9}, {6, 0}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_OR_B;
  input_mode.hole_mode = HoleMode::WITH_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 34);
  EXPECT_EQ(result.intersections_data.size(), 24);
  EXPECT_EQ(result.offsets.size(), 7);

  if (DO_DRAW) {
    draw_curve("Complex A Union B", points_a, points_b, result);
  }
}

void complex_A_OR_B_without_holes_test()
{
  /**
   * This is a replica of Fig. 16 from
   * Greiner, Günther; Kai Hormann (1998). "Efficient clipping of arbitrary polygons". ACM
   * Transactions on Graphics. 17 (2): 71–83.
   */
  const Array<float2> points_a = {{14, 1}, {0, 5}, {14, 10}, {5, 6}, {14, 6}, {5, 5}};
  const Array<float2> points_b = {{9, 13}, {13, 0}, {9, 9}, {6, 0}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_OR_B;
  input_mode.hole_mode = HoleMode::WITHOUT_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 17);
  EXPECT_EQ(result.intersections_data.size(), 10);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Complex A Union B without holes", points_a, points_b, result);
  }
}

void complex_A_NOT_B_test()
{
  /**
   * This is a replica of Fig. 16 from
   * Greiner, Günther; Kai Hormann (1998). "Efficient clipping of arbitrary polygons". ACM
   * Transactions on Graphics. 17 (2): 71–83.
   */
  const Array<float2> points_a = {{14, 1}, {0, 5}, {14, 10}, {5, 6}, {14, 6}, {5, 5}};
  const Array<float2> points_b = {{9, 13}, {13, 0}, {9, 9}, {6, 0}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_NOT_B;
  input_mode.hole_mode = HoleMode::WITH_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 30);
  EXPECT_EQ(result.intersections_data.size(), 24);
  EXPECT_EQ(result.offsets.size(), 8);

  if (DO_DRAW) {
    draw_curve("Complex A without B", points_a, points_b, result);
  }
}

void complex_B_NOT_A_test()
{
  /**
   * This is a replica of Fig. 16 from
   * Greiner, Günther; Kai Hormann (1998). "Efficient clipping of arbitrary polygons". ACM
   * Transactions on Graphics. 17 (2): 71–83.
   */
  const Array<float2> points_a = {{14, 1}, {0, 5}, {14, 10}, {5, 6}, {14, 6}, {5, 5}};
  const Array<float2> points_b = {{9, 13}, {13, 0}, {9, 9}, {6, 0}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::B_NOT_A;
  input_mode.hole_mode = HoleMode::WITH_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 28);
  EXPECT_EQ(result.intersections_data.size(), 24);
  EXPECT_EQ(result.offsets.size(), 8);

  if (DO_DRAW) {
    draw_curve("Complex B without A", points_a, points_b, result);
  }
}

void last_segment_interection_test()
{
  /**
   * These shapes are designed to test the following:
   *   1: Intersection with the last segment to others.
   *   2: Getting the next intersection point through a full loop from last segment to the first.
   *   3: Sorting multiple intersection points on the same segment.
   *   4: Self intersection in one of the shapes.
   */
  const Array<float2> points_a = {{0, 5}, {0, 0}, {7, 0}, {7, 5}};
  const Array<float2> points_b = {{2, 3}, {0, 7}, {3, 7}, {5, 4}, {6, 6}, {3, 4}, {2, 6}};
  InputMode input_mode;
  input_mode.boolean_mode = BooleanMode::A_NOT_B;
  input_mode.hole_mode = HoleMode::WITH_HOLES;
  BooleanResult result = polygonboolean::curve_boolean_calc(input_mode, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 13);
  EXPECT_EQ(result.intersections_data.size(), 6);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_curve("Last segment loop", points_a, points_b, result);
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

TEST(polygonboolean, Simple_Intersection)
{
  simple_intersection_test();
}

TEST(polygonboolean, Simple_Union_With_Hole)
{
  simple_union_with_hole_test();
}

TEST(polygonboolean, Simple_Union_Without_Hole)
{
  simple_union_without_hole_test();
}

TEST(polygonboolean, Complex_A_AND_B)
{
  complex_A_AND_B_test();
}

TEST(polygonboolean, Complex_A_OR_B)
{
  complex_A_OR_B_test();
}

TEST(polygonboolean, Complex_A_OR_B_Without_Holes)
{
  complex_A_OR_B_without_holes_test();
}

TEST(polygonboolean, Complex_A_NOT_B)
{
  complex_A_NOT_B_test();
}

TEST(polygonboolean, Complex_B_NOT_A)
{
  complex_B_NOT_A_test();
}

TEST(polygonboolean, Last_Segment_Interection)
{
  last_segment_interection_test();
}

}  // namespace blender::polygonboolean
