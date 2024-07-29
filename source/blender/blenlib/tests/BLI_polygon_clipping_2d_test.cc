/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "testing/testing.h"

#include "MEM_guardedalloc.h"

#include "BLI_array.hh"
#include "BLI_bounds.hh"
#include "BLI_offset_indices.hh"

#include <fstream>
#include <iostream>
#include <sstream>
#include <type_traits>

/* Should tests draw their output to an HTML file? */
#define DO_DRAW 0

#include "BLI_polygon_clipping_2d.hh"

namespace blender::polygonboolean {

static void CSS_setup_style(std::ofstream &f)
{
  constexpr int border_width = 5;
  constexpr int stroke_width = 3;
  constexpr int stroke_dasharray = 15;

  f << "div {\n"
       "  border: "
    << border_width
    << "px solid black;\n"
       "  text-align: center;\n"
       "}\n"
       "\n";

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
       "}\n"
       "\n";

  f << ".cut-A {\n"
       "  fill: none;\n"
       "  stroke: blue;\n"
       "  stroke-width: "
    << stroke_width
    << "px;\n"
       "  stroke-dasharray: "
    << stroke_dasharray
    << "px;\n"
       "}\n";
  f << ".cut-B {\n"
       "  fill: red;\n"
       "  stroke: red;\n"
       "  fill-opacity: 0.25;\n"
       "  stroke-width: "
    << stroke_width
    << "px;\n"
       "  stroke-dasharray: "
    << stroke_dasharray
    << "px;\n"
       "}\n";
  f << ".cut-C {\n"
       "  fill: none;\n"
       "  stroke: black;\n"
       "  stroke-width: "
    << stroke_width + 1
    << "px;\n"
       "}\n";
}

#define SX(x) ((x - topleft[0]) * scale)
#define SY(y) ((topleft[1] - y) * scale)

static void SVG_add_polygon(std::ofstream &f,
                            const std::string &class_name,
                            const Span<float2> points,
                            const float2 &topleft,
                            const float scale)
{
  f << "<polygon class = \"" << class_name << "\" points = \"";
  for (const int i : points.index_range()) {
    const float2 &point = points[i];
    if (i != 0) {
      f << ", ";
    }
    f << SX(point[0]) << "," << SY(point[1]);
  }
  f << "\"/>\n";
}

static void SVG_add_polygons_as_path(std::ofstream &f,
                                     const std::string &class_name,
                                     const Span<float2> points,
                                     const OffsetIndices<int> points_by_polygon,
                                     const float2 &topleft,
                                     const float scale)
{
  f << "<path class = \"" << class_name << "\" d = \"";
  for (const int polygon_id : points_by_polygon.index_range()) {
    const IndexRange vert_ids = points_by_polygon[polygon_id];
    if (polygon_id != 0) {
      f << " ";
    }

    f << "M ";
    for (const int i : vert_ids) {
      const float2 &point = points[i];
      const int j = i - vert_ids.first();

      if (j == 1) {
        f << " L ";
      }
      else if (j != 0) {
        f << ", ";
      }
      f << SX(point[0]) << "," << SY(point[1]);
    }
    f << " Z";
  }

  f << "\"";

  f << " fill-rule=\"evenodd\"";

  f << "/>\n";
}

static void SVG_add_line(std::ofstream &f,
                         const std::string &class_name,
                         const Span<float2> points,
                         const float2 &topleft,
                         const float scale)
{
  f << "<path class = \"" << class_name << "\" d = \"";

  f << "M ";
  for (const int i : points.index_range()) {
    const float2 &point = points[i];

    if (i == 1) {
      f << " L ";
    }
    else if (i != 0) {
      f << ", ";
    }
    f << SX(point[0]) << "," << SY(point[1]);
  }

  f << "\"";

  f << "/>\n";
}

static void SVG_add_lines(std::ofstream &f,
                          const std::string &class_name,
                          const Span<float2> points,
                          const OffsetIndices<int> points_by_polygon,
                          const float2 &topleft,
                          const float scale)
{
  f << "<path class = \"" << class_name << "\" d = \"";
  for (const int polygon_id : points_by_polygon.index_range()) {
    const IndexRange vert_ids = points_by_polygon[polygon_id];
    if (polygon_id != 0) {
      f << " ";
    }

    f << "M ";
    for (const int i : vert_ids) {
      const float2 &point = points[i];
      const int j = i - vert_ids.first();

      if (j == 1) {
        f << " L ";
      }
      else if (j != 0) {
        f << ", ";
      }
      f << SX(point[0]) << "," << SY(point[1]);
    }
  }

  f << "\"";

  f << "/>\n";
}

static bool draw_append = false; /* Will be set to true after first call. */

void draw_polygons(const std::string &label,
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

  const Bounds<float2> bound = *bounds::merge(bounds::min_max(curve_a), bounds::min_max(curve_b));
  const float2 vmin = bound.min;
  const float2 vmax = bound.max;
  const float draw_margin = ((vmax.x - vmin.x) + (vmax.y - vmin.y)) * 0.05;
  const float minx = vmin.x - draw_margin;
  const float maxx = vmax.x + draw_margin;
  const float miny = vmin.y - draw_margin;
  const float maxy = vmax.y + draw_margin;

  const float2 topleft = float2(minx, maxy);

  const float width = maxx - minx;
  const float height = maxy - miny;
  const float aspect = height / width;
  int view_width = max_draw_width;
  int view_height = int(view_width * aspect);
  if (view_height > max_draw_height) {
    view_height = max_draw_height;
    view_width = int(view_height / aspect);
  }
  const float scale = view_width / width;

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

    CSS_setup_style(f);

    f << "</style>\n";
  }

  f << "<div>\n";
  f << "<h1>" << label << "</h1>\n";

  f << "<svg width=\"" << view_width << "\" height=\"" << view_height << "\">\n";

  SVG_add_polygon(f, "polygon-A", curve_a, topleft, scale);
  SVG_add_polygon(f, "polygon-B", curve_b, topleft, scale);

  const Span<float2> points = interpolate_attribute_from_ab_result<float2>(
      curve_a, curve_b, result);
  const OffsetIndices<int> points_by_polygon = OffsetIndices<int>(result.offsets);

  if (points_by_polygon.size() == 1) {
    SVG_add_polygon(f, "polygon-C", points, topleft, scale);
  }
  else {
    SVG_add_polygons_as_path(f, "polygon-C", points, points_by_polygon, topleft, scale);
  }

  f << "</div>\n";

  draw_append = true;
}

void draw_cut(const std::string &label,
              const bool is_a_cyclic,
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

  const Bounds<float2> bound = *bounds::merge(bounds::min_max(curve_a), bounds::min_max(curve_b));
  const float2 vmin = bound.min;
  const float2 vmax = bound.max;
  const float draw_margin = ((vmax.x - vmin.x) + (vmax.y - vmin.y)) * 0.05;
  const float minx = vmin.x - draw_margin;
  const float maxx = vmax.x + draw_margin;
  const float miny = vmin.y - draw_margin;
  const float maxy = vmax.y + draw_margin;

  const float2 topleft = float2(minx, maxy);

  const float width = maxx - minx;
  const float height = maxy - miny;
  const float aspect = height / width;
  int view_width = max_draw_width;
  int view_height = int(view_width * aspect);
  if (view_height > max_draw_height) {
    view_height = max_draw_height;
    view_width = int(view_height / aspect);
  }
  const float scale = view_width / width;

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

    CSS_setup_style(f);

    f << "</style>\n";
  }

  f << "<div>\n";
  f << "<h1>" << label << "</h1>\n";

  f << "<svg width=\"" << view_width << "\" height=\"" << view_height << "\">\n";

  if (is_a_cyclic) {
    SVG_add_polygon(f, "cut-A", curve_a, topleft, scale);
  }
  else {
    SVG_add_line(f, "cut-A", curve_a, topleft, scale);
  }
  SVG_add_polygon(f, "cut-B", curve_b, topleft, scale);

  const Span<float2> points = interpolate_attribute_from_a_result<float2>(curve_a, result);
  const OffsetIndices<int> points_by_polygon = OffsetIndices<int>(result.offsets);

  if (points_by_polygon.size() == 1) {
    SVG_add_line(f, "cut-C", points, topleft, scale);
  }
  else {
    SVG_add_lines(f, "cut-C", points, points_by_polygon, topleft, scale);
  }

  f << "</div>\n";

  draw_append = true;
}

#undef SX
#undef SY

void squares_A_AND_B_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_AND_B, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 4);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_polygons("Squares A intersection B", points_a, points_b, result);
  }
}

void squares_A_OR_B_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_OR_B, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 8);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_polygons("Squares A union B", points_a, points_b, result);
  }
}

void squares_A_NOT_B_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_NOT_B, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 6);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_polygons("Squares A without B", points_a, points_b, result);
  }
}

void squares_B_NOT_A_test()
{
  const Array<float2> points_a = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Array<float2> points_b = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::B_NOT_A, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 6);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_polygons("Squares B without A", points_a, points_b, result);
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
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_AND_B, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 6);
  EXPECT_EQ(result.intersections_data.size(), 4);
  EXPECT_EQ(result.offsets.size(), 3);

  if (DO_DRAW) {
    draw_polygons("Simple Intersection", points_a, points_b, result);
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
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_OR_B, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 11);
  EXPECT_EQ(result.intersections_data.size(), 4);
  EXPECT_EQ(result.offsets.size(), 3);

  if (DO_DRAW) {
    draw_polygons("Simple Union With Hole", points_a, points_b, result);
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
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_OR_B, HoleMode::WITHOUT_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 8);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_polygons("Simple Union Without Hole", points_a, points_b, result);
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
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_AND_B, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 24);
  EXPECT_EQ(result.intersections_data.size(), 24);
  EXPECT_EQ(result.offsets.size(), 7);

  if (DO_DRAW) {
    draw_polygons("Complex A Intersection B", points_a, points_b, result);
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
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_OR_B, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 34);
  EXPECT_EQ(result.intersections_data.size(), 24);
  EXPECT_EQ(result.offsets.size(), 7);

  if (DO_DRAW) {
    draw_polygons("Complex A Union B", points_a, points_b, result);
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
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_OR_B, HoleMode::WITHOUT_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 17);
  EXPECT_EQ(result.intersections_data.size(), 10);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_polygons("Complex A Union B without holes", points_a, points_b, result);
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
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_NOT_B, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 30);
  EXPECT_EQ(result.intersections_data.size(), 24);
  EXPECT_EQ(result.offsets.size(), 8);

  if (DO_DRAW) {
    draw_polygons("Complex A without B", points_a, points_b, result);
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
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::B_NOT_A, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 28);
  EXPECT_EQ(result.intersections_data.size(), 24);
  EXPECT_EQ(result.offsets.size(), 8);

  if (DO_DRAW) {
    draw_polygons("Complex B without A", points_a, points_b, result);
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
  BooleanResult result = polygonboolean::curve_boolean_calc(
      {BooleanMode::A_NOT_B, HoleMode::WITH_HOLES}, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 13);
  EXPECT_EQ(result.intersections_data.size(), 6);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_polygons("Last segment loop", points_a, points_b, result);
  }
}

void simple_cut_test()
{
  /**
   * With cut the first curve A does not loop.
   */
  const Array<float2> points_a = {{5, 7}, {3, 6}, {0, 2}, {0, 0}};
  const Array<float2> points_b = {{1, 6}, {3, 4}, {3, 1}, {0, 4}, {2, 3}};
  BooleanResult result = polygonboolean::curve_boolean_cut(false, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 8);
  EXPECT_EQ(result.intersections_data.size(), 4);
  EXPECT_EQ(result.offsets.size(), 4);

  if (DO_DRAW) {
    draw_cut("Simple cut", false, points_a, points_b, result);
  }
}

void simple_cut_2_test()
{
  /**
   * With cut the first curve A does not loop.
   */
  const Array<float2> points_a = {{5, 5}, {3, 5}, {1, 3}, {1, 1}};
  const Array<float2> points_b = {{5, 6}, {6, 5}, {1, 0}, {0, 1}};
  BooleanResult result = polygonboolean::curve_boolean_cut(false, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 4);
  EXPECT_EQ(result.intersections_data.size(), 2);
  EXPECT_EQ(result.offsets.size(), 2);

  if (DO_DRAW) {
    draw_cut("Simple cut 2", false, points_a, points_b, result);
  }
}

void simple_cut_3_test()
{
  /**
   * With cut the first curve A does not loop.
   */
  const Array<float2> points_a = {{6, 8}, {4, 7}, {1, 3}, {1, 1}};
  const Array<float2> points_b = {
      {3, 7}, {5, 5}, {1, 0}, {0, 4}, {2, 3}, {1, 5}, {3, 4}, {2, 6}, {4, 5}};
  BooleanResult result = polygonboolean::curve_boolean_cut(false, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 9);
  EXPECT_EQ(result.intersections_data.size(), 7);
  EXPECT_EQ(result.offsets.size(), 5);

  if (DO_DRAW) {
    draw_cut("Simple cut 3", false, points_a, points_b, result);
  }
}

void simple_cut_4_test()
{
  /**
   * With cut the first curve A does not loop.
   */
  const Array<float2> points_a = {{6, 7}, {4, 6}, {1, 2}, {1, 0}};
  const Array<float2> points_b = {
      {0, 4}, {2, 2}, {7, 8}, {3, 7}, {4, 5}, {2, 6}, {3, 4}, {1, 5}, {2, 3}};
  BooleanResult result = polygonboolean::curve_boolean_cut(false, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 9);
  EXPECT_EQ(result.intersections_data.size(), 7);
  EXPECT_EQ(result.offsets.size(), 5);

  if (DO_DRAW) {
    draw_cut("Simple cut 4", false, points_a, points_b, result);
  }
}

void cyclical_cut_test()
{
  const Array<float2> points_a = {{6, 5}, {4, 5}, {1, 2}, {1, 0}};
  const Array<float2> points_b = {{1, 4}, {3, 1}, {5, 3}, {2, 5}, {3, 3}};
  BooleanResult result = polygonboolean::curve_boolean_cut(true, points_a, points_b);
  EXPECT_TRUE(result.valid_geometry);
  EXPECT_EQ(result.verts.size(), 10);
  EXPECT_EQ(result.intersections_data.size(), 6);
  EXPECT_EQ(result.offsets.size(), 4);

  if (DO_DRAW) {
    draw_cut("Cyclical cut", true, points_a, points_b, result);
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

TEST(polygonboolean, Simple_Cut)
{
  simple_cut_test();
}

TEST(polygonboolean, Simple_Cut_2)
{
  simple_cut_2_test();
}

TEST(polygonboolean, Simple_Cut_3)
{
  simple_cut_3_test();
}

TEST(polygonboolean, Simple_Cut_4)
{
  simple_cut_4_test();
}

TEST(polygonboolean, Cyclical_Cut)
{
  cyclical_cut_test();
}

}  // namespace blender::polygonboolean
