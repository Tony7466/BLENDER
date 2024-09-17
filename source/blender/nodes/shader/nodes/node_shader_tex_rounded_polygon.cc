/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "node_shader_util.hh"
#include "node_util.hh"

#include "BKE_texture.h"

#include "BLI_noise.hh"

#include "NOD_multi_function.hh"

#include "RNA_access.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_shader_tex_raiko_base_cc {

NODE_STORAGE_FUNCS(NodeTexRaikoBase)

static void sh_node_tex_raiko_base_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.use_custom_socket_order();

  b.add_output<decl::Float>("R_sphere Field").no_muted_links();
  b.add_output<decl::Float>("R_gon Parameter Field").no_muted_links();
  b.add_output<decl::Float>("Max Unit Parameter").no_muted_links();

  b.add_input<decl::Vector>("Vector")
      .hide_value()
      .implicit_field(implicit_field_inputs::position)
      .description("XYZ components of the input vector");
  b.add_input<decl::Float>("W").min(-1000.0f).max(1000.0f).default_value(0.0f).description(
      "W component of the input vector");
  b.add_input<decl::Float>("Scale").min(-1000.0f).max(1000.0f).default_value(1.0f).description(
      "Factor by which the input vector is scaled");

  /* Panel for r-sphere inputs. */
  PanelDeclarationBuilder &r_sphere =
      b.add_panel("R-sphere")
          .default_closed(false)
          .draw_buttons([](uiLayout *layout, bContext * /*C*/, PointerRNA *ptr) {
            uiItemR(
                layout, ptr, "elliptical_corners", UI_ITEM_R_SPLIT_EMPTY_NAME, nullptr, ICON_NONE);
          });
  r_sphere.add_input<decl::Float>("R_gon Sides")
      .min(2.0f)
      .max(1000.0f)
      .default_value(5.0f)
      .description("Number of R-gon base sides");
  r_sphere.add_input<decl::Float>("R_gon Roundness")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .description("R-gon base corner roundness");
  r_sphere.add_input<decl::Float>("R_gon Exponent")
      .min(0.0f)
      .max(1000.0f)
      .default_value(2.0f)
      .description("P-norm exponent of the R-gon base");
  r_sphere.add_input<decl::Float>("Sphere Exponent")
      .min(0.0f)
      .max(1000.0f)
      .default_value(2.0f)
      .description("P-norm exponent of the sphere");
}

static void node_shader_buts_tex_raiko_base(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(
      layout, ptr, "normalize_r_gon_parameter", UI_ITEM_R_SPLIT_EMPTY_NAME, nullptr, ICON_NONE);
}

static void node_shader_init_tex_raiko_base(bNodeTree * /*ntree*/, bNode *node)
{
  NodeTexRaikoBase *tex = MEM_cnew<NodeTexRaikoBase>(__func__);
  BKE_texture_mapping_default(&tex->base.tex_mapping, TEXMAP_TYPE_POINT);
  BKE_texture_colormapping_default(&tex->base.color_mapping);
  tex->normalize_r_gon_parameter = false;
  tex->elliptical_corners = false;

  node->storage = tex;
}

static const char *gpu_shader_get_name()
{
  return "node_tex_raiko_base";
}

static int node_shader_gpu_tex_raiko_base(GPUMaterial *mat,
                                          bNode *node,
                                          bNodeExecData * /*execdata*/,
                                          GPUNodeStack *in,
                                          GPUNodeStack *out)
{
  node_shader_gpu_default_tex_coord(mat, node, &in[0].link);
  node_shader_gpu_tex_mapping(mat, node, in, out);

  const NodeTexRaikoBase &storage = node_storage(*node);
  float normalize_r_gon_parameter = storage.normalize_r_gon_parameter;
  float elliptical_corners = storage.elliptical_corners;
  float calculate_r_gon_parameter_field = out[1].hasoutput;
  float calculate_max_unit_parameter = out[2].hasoutput;

  const char *name = gpu_shader_get_name();

  return GPU_stack_link(mat,
                        node,
                        name,
                        in,
                        out,
                        GPU_constant(&normalize_r_gon_parameter),
                        GPU_constant(&elliptical_corners),
                        GPU_constant(&calculate_r_gon_parameter_field),
                        GPU_constant(&calculate_max_unit_parameter));
}

static void node_shader_update_tex_raiko_base(bNodeTree *ntree, bNode *node)
{
  (void)ntree;

  bNodeSocket *inR_gonSidesSock = bke::node_find_socket(node, SOCK_IN, "R_gon Sides");
  bNodeSocket *inR_gonRoundnessSock = bke::node_find_socket(node, SOCK_IN, "R_gon Roundness");
  bNodeSocket *inR_gonExponentSock = bke::node_find_socket(node, SOCK_IN, "R_gon Exponent");
  bNodeSocket *inSphereExponentSock = bke::node_find_socket(node, SOCK_IN, "Sphere Exponent");

  bNodeSocket *outR_sphereFieldSock = bke::node_find_socket(node, SOCK_OUT, "R_sphere Field");
  bNodeSocket *outR_gonParameterFieldSock = bke::node_find_socket(
      node, SOCK_OUT, "R_gon Parameter Field");

  node_sock_label(inR_gonSidesSock, "Sides");
  node_sock_label(inR_gonRoundnessSock, "Roundness");
  node_sock_label(inR_gonExponentSock, "Base Convexity");
  node_sock_label(inSphereExponentSock, "Sphere Convexity");

  node_sock_label(outR_sphereFieldSock, "R-sphere Field");
  node_sock_label(outR_gonParameterFieldSock, "R-gon Parameter Field");
}

float chebychev_norm(float coord)
{
  return fabsf(coord);
}

float chebychev_norm(float2 coord)
{
  return math::max(fabsf(coord.x), fabsf(coord.y));
}

float chebychev_norm(float3 coord)
{
  return math::max(fabsf(coord.x), math::max(fabsf(coord.y), fabsf(coord.z)));
}

float p_norm(float coord)
{
  return fabsf(coord);
}

float p_norm(float2 coord, float exponent)
{
  /* Use Chebychev norm instead of p-norm for high exponent values to avoid going out of the
   * floating point representable range. */
  return (exponent > 32.0f) ? chebychev_norm(coord) :
                              powf(powf(fabsf(coord.x), exponent) + powf(fabsf(coord.y), exponent),
                                   1.0f / exponent);
}

float p_norm(float3 coord, float exponent)
{
  /* Use Chebychev norm instead of p-norm for high exponent values to avoid going out of the
   * floating point representable range. */
  return (exponent > 32.0f) ?
             chebychev_norm(coord) :
             powf(powf(fabsf(coord.x), exponent) + powf(fabsf(coord.y), exponent) +
                      powf(fabsf(coord.z), exponent),
                  1.0f / exponent);
}

float3 calculate_out_fields_2d_full_roundness_irregular_elliptical(

    bool calculate_r_gon_parameter_field,
    bool normalize_r_gon_parameter,
    float r_gon_sides,
    float2 coord,
    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    float r_gon_parameter_2d = 0.0f;
    if (calculate_r_gon_parameter_field) {
      r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
      if (ref_A_coord < ref_A_angle_bisector) {
        r_gon_parameter_2d *= -1.0f;
      }
      if (normalize_r_gon_parameter) {
        r_gon_parameter_2d /= ref_A_angle_bisector;
      }
    }
    return float3(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector);
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0f;
    float r_gon_parameter_2d = 0.0f;
    float max_unit_parameter_2d = 0.0f;

    float l_basis_vector_1 = tan(ref_A_angle_bisector);
    /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
     * precision issues. Change to double if necessary */
    float l_basis_vector_2 = sin(last_angle_bisector_A_x_axis) *
                             sqrtf(math::square(tan(ref_A_angle_bisector)) + 1.0f);
    float2 ellipse_center =
        float2(cos(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector),
               sin(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector)) -
        l_basis_vector_2 *
            float2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
    float2 transformed_direction_vector = float2(
        cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
            (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
            (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float2 transformed_origin = float2(
        (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
         ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
            (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        -(ellipse_center.y * sin(ref_A_angle_bisector) +
          ellipse_center.x * cos(ref_A_angle_bisector)) /
            (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float l_coord_R_l_angle_bisector_2d =
        (-(transformed_direction_vector.x * transformed_origin.x +
           transformed_direction_vector.y * transformed_origin.y) +
         sqrtf(math::square(transformed_direction_vector.x * transformed_origin.x +
                            transformed_direction_vector.y * transformed_origin.y) -
               (math::square(transformed_direction_vector.x) +
                math::square(transformed_direction_vector.y)) *
                   (math::square(transformed_origin.x) + math::square(transformed_origin.y) -
                    1.0f))) /
        (math::square(transformed_direction_vector.x) +
         math::square(transformed_direction_vector.y));
    l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
    if (nearest_ref_MSA_coord < 0.0f) {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= last_angle_bisector_A_x_axis *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      max_unit_parameter_2d = last_angle_bisector_A_x_axis *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
    }
    else {
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      max_unit_parameter_2d = ref_A_angle_bisector;
    }
    return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
  }
}

float3 calculate_out_fields_2d_irregular_elliptical(bool calculate_r_gon_parameter_field,
                                                    bool calculate_max_unit_parameter,
                                                    bool normalize_r_gon_parameter,
                                                    float r_gon_sides,
                                                    float r_gon_roundness,
                                                    float2 coord,
                                                    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1.0f - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1.0f - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start))
  {
    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d *
                                    tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                ref_A_bevel_start;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) + ref_A_bevel_start;
      }
      return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
      float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                              sin(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start =
          cos(nearest_ref_SA_coord) * l_circle_center +
          sqrtf(math::square(cos(nearest_ref_SA_coord) * l_circle_center) +
                math::square(l_circle_radius) - math::square(l_circle_center));
      l_angle_bisector_2d = cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                            l_coord_R_l_bevel_start;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(ref_A_angle_bisector - ref_A_bevel_start) +
                             ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d *
                                    tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                ref_A_bevel_start;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) + ref_A_bevel_start;
      }
      return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                            l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d *
                             tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                          inner_last_bevel_start_A_x_axis) +
                                inner_last_bevel_start_A_x_axis *
                                    l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                    inner_last_bevel_start_A_x_axis) +
                                inner_last_bevel_start_A_x_axis *
                                    l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
      return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_basis_vector_1 = r_gon_roundness * tan(ref_A_angle_bisector);
      float l_basis_vector_2 = r_gon_roundness * sin(last_angle_bisector_A_x_axis) *
                               sqrtf(math::square(tan(ref_A_angle_bisector)) + 1.0f);
      float2 ellipse_center =
          float2(cos(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start),
                 sin(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start)) -
          l_basis_vector_2 *
              float2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
      float2 transformed_direction_vector = float2(
          cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float2 transformed_origin = float2(
          (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
           ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          -(ellipse_center.y * sin(ref_A_angle_bisector) +
            ellipse_center.x * cos(ref_A_angle_bisector)) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float l_coord_R_l_angle_bisector_2d =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrtf(math::square(transformed_direction_vector.x * transformed_origin.x +
                              transformed_direction_vector.y * transformed_origin.y) -
                 (math::square(transformed_direction_vector.x) +
                  math::square(transformed_direction_vector.y)) *
                     (math::square(transformed_origin.x) + math::square(transformed_origin.y) -
                      1.0f))) /
          (math::square(transformed_direction_vector.x) +
           math::square(transformed_direction_vector.y));
      l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
      if (nearest_ref_MSA_coord < 0.0f) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(last_angle_bisector_A_x_axis -
                                                               inner_last_bevel_start_A_x_axis)) +
                               (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                   l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                            inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                      inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      else {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - nearest_ref_MSA_coord;
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
      }
      return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
}

float3 calculate_out_fields_2d_full_roundness_irregular_circular(

    bool calculate_r_gon_parameter_field,
    bool calculate_max_unit_parameter,
    bool normalize_r_gon_parameter,
    float r_gon_sides,
    float2 coord,
    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = float2(cos(last_angle_bisector_A_x_axis) -
                                         l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
                                     l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
                                         sin(last_angle_bisector_A_x_axis));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius *
                                      float2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d *
                                    tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                x_axis_A_outer_last_bevel_start;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                x_axis_A_outer_last_bevel_start;
      }
      return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      float r_gon_parameter_2d = 0.0f;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return float3(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector);
    }
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0f;
    float r_gon_parameter_2d = 0.0f;
    float max_unit_parameter_2d = 0.0f;

    float l_coord_R_l_last_angle_bisector_2d =
        sin(nearest_ref_MSA_coord) * last_circle_center.y +
        cos(nearest_ref_MSA_coord) * last_circle_center.x +
        sqrtf(math::square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                           cos(nearest_ref_MSA_coord) * last_circle_center.x) +
              math::square(l_last_circle_radius) - math::square(last_circle_center.x) -
              math::square(last_circle_center.y));
    l_angle_bisector_2d = (cos(ref_A_angle_bisector) * l_projection_2d) /
                          (cos(last_angle_bisector_A_x_axis) * l_coord_R_l_last_angle_bisector_2d);
    if (nearest_ref_MSA_coord < 0.0f) {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= last_angle_bisector_A_x_axis *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      max_unit_parameter_2d = last_angle_bisector_A_x_axis *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
    }
    else {
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector -
                                                             x_axis_A_outer_last_bevel_start)) +
                             x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d *
                                    tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                x_axis_A_outer_last_bevel_start;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                x_axis_A_outer_last_bevel_start;
      }
    }
    return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
  }
}

float3 calculate_out_fields_2d_irregular_circular(bool calculate_r_gon_parameter_field,
                                                  bool calculate_max_unit_parameter,
                                                  bool normalize_r_gon_parameter,
                                                  float r_gon_sides,
                                                  float r_gon_roundness,
                                                  float2 coord,
                                                  float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1.0f - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1.0f - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = float2(
      (cos(inner_last_bevel_start_A_x_axis) /
       cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)) -
          l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
          (sin(inner_last_bevel_start_A_x_axis) /
           cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius *
                                      float2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if (((ref_A_coord >= ref_A_bevel_start) &&
         (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) ||
        (x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) ||
        (x_axis_A_coord < ref_A_bevel_start))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
          (x_axis_A_coord < ref_A_angle_bisector))
      {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
        }
      }
      else {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
      }
      return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
      float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                              sin(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start =
          cos(nearest_ref_SA_coord) * l_circle_center +
          sqrtf(math::square(cos(nearest_ref_SA_coord) * l_circle_center) +
                math::square(l_circle_radius) - math::square(l_circle_center));
      l_angle_bisector_2d = cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                            l_coord_R_l_bevel_start;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(ref_A_angle_bisector - ref_A_bevel_start) +
                             ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d *
                                    tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                ref_A_bevel_start;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) + ref_A_bevel_start;
      }
      return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                            l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d *
                             tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                          inner_last_bevel_start_A_x_axis) +
                                inner_last_bevel_start_A_x_axis *
                                    l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                    inner_last_bevel_start_A_x_axis) +
                                inner_last_bevel_start_A_x_axis *
                                    l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
      return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_coord_R_l_last_angle_bisector_2d =
          sin(nearest_ref_MSA_coord) * last_circle_center.y +
          cos(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrtf(math::square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                             cos(nearest_ref_MSA_coord) * last_circle_center.x) +
                math::square(l_last_circle_radius) - math::square(last_circle_center.x) -
                math::square(last_circle_center.y));
      l_angle_bisector_2d = (cos(ref_A_angle_bisector) * l_projection_2d) /
                            (cos(last_angle_bisector_A_x_axis) *
                             l_coord_R_l_last_angle_bisector_2d);
      if (nearest_ref_MSA_coord < 0.0f) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(last_angle_bisector_A_x_axis -
                                                               inner_last_bevel_start_A_x_axis)) +
                               (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                   l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                            inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                      inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      else {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector -
                                                               x_axis_A_outer_last_bevel_start)) +
                               x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
        }
      }
      return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
}

float3 calculate_out_fields_2d(bool calculate_r_gon_parameter_field,
                               bool calculate_max_unit_parameter,
                               bool normalize_r_gon_parameter,
                               bool elliptical_corners,
                               float r_gon_sides,
                               float r_gon_roundness,
                               float r_gon_exponent,
                               float2 coord)
{
  float l_projection_2d = p_norm(coord, r_gon_exponent);

  if (fractf(r_gon_sides) == 0.0f) {
    float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
    float ref_A_angle_bisector = M_PI_F / r_gon_sides;
    float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
    float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
    float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

    if (r_gon_roundness == 0.0f) {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter && (r_gon_sides != 2.0f)) {
          r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = (r_gon_sides != 2.0f) ? tan(ref_A_angle_bisector) : 0.0f;
      }
      return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    if (r_gon_roundness == 1.0f) {
      float r_gon_parameter_2d = 0.0f;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return float3(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector);
    }
    else {
      float ref_A_bevel_start = ref_A_angle_bisector -
                                atan((1.0f - r_gon_roundness) * tan(ref_A_angle_bisector));

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;

        float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
        float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                                sin(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start =
            cos(nearest_ref_SA_coord) * l_circle_center +
            sqrtf(math::square(cos(nearest_ref_SA_coord) * l_circle_center) +
                  math::square(l_circle_radius) - math::square(l_circle_center));
        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_bevel_start) /
                              l_coord_R_l_bevel_start;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
        return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
      else {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;

        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
        return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
    }
  }
  else {
    if (r_gon_roundness == 0.0f) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis) {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;

        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector);
        }
        return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
      else {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;

        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis);
        }
        return float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
    }
    if (r_gon_roundness == 1.0f) {
      if (elliptical_corners) {
        return calculate_out_fields_2d_full_roundness_irregular_elliptical(

            calculate_r_gon_parameter_field,
            normalize_r_gon_parameter,
            r_gon_sides,
            coord,
            l_projection_2d);
      }
      else {
        return calculate_out_fields_2d_full_roundness_irregular_circular(

            calculate_r_gon_parameter_field,
            calculate_max_unit_parameter,
            normalize_r_gon_parameter,
            r_gon_sides,
            coord,
            l_projection_2d);
      }
    }
    else {
      if (elliptical_corners) {
        return calculate_out_fields_2d_irregular_elliptical(calculate_r_gon_parameter_field,
                                                            calculate_max_unit_parameter,
                                                            normalize_r_gon_parameter,
                                                            r_gon_sides,
                                                            r_gon_roundness,
                                                            coord,
                                                            l_projection_2d);
      }
      else {
        return calculate_out_fields_2d_irregular_circular(calculate_r_gon_parameter_field,
                                                          calculate_max_unit_parameter,
                                                          normalize_r_gon_parameter,
                                                          r_gon_sides,
                                                          r_gon_roundness,
                                                          coord,
                                                          l_projection_2d);
      }
    }
  }
}

float3 calculate_out_fields_4d(bool calculate_r_gon_parameter_field,
                               bool calculate_max_unit_parameter,
                               bool normalize_r_gon_parameter,
                               bool elliptical_corners,
                               float r_gon_sides,
                               float r_gon_roundness,
                               float r_gon_exponent,
                               float sphere_exponent,
                               float4 coord)
{
  float3 out_fields = calculate_out_fields_2d(calculate_r_gon_parameter_field,
                                              calculate_max_unit_parameter,
                                              normalize_r_gon_parameter,
                                              elliptical_corners,
                                              r_gon_sides,
                                              r_gon_roundness,
                                              r_gon_exponent,
                                              float2(coord.x, coord.y));
  out_fields.x = p_norm(float3(out_fields.x, coord.z, coord.w), sphere_exponent);
  return out_fields;
}

class RaikoBaseFunction : public mf::MultiFunction {
 private:
  bool normalize_r_gon_parameter_;
  bool elliptical_corners_;

  mf::Signature signature_;

 public:
  RaikoBaseFunction(bool normalize_r_gon_parameter, bool elliptical_corners)
      : normalize_r_gon_parameter_(normalize_r_gon_parameter),
        elliptical_corners_(elliptical_corners)
  {
    signature_ = create_signature();
    this->set_signature(&signature_);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"raiko_base", signature};

    builder.single_input<float3>("Vector");
    builder.single_input<float>("W");
    builder.single_input<float>("Scale");

    builder.single_input<float>("R_gon Sides");
    builder.single_input<float>("R_gon Roundness");
    builder.single_input<float>("R_gon Exponent");
    builder.single_input<float>("Sphere Exponent");

    builder.single_output<float>("R_sphere Field", mf::ParamFlag::SupportsUnusedOutput);
    builder.single_output<float>("R_gon Parameter Field", mf::ParamFlag::SupportsUnusedOutput);
    builder.single_output<float>("Max Unit Parameter", mf::ParamFlag::SupportsUnusedOutput);

    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;

    const VArray<float3> &coord = params.readonly_single_input<float3>(param++, "Vector");
    const VArray<float> &in_w = params.readonly_single_input<float>(param++, "W");
    const VArray<float> &scale = params.readonly_single_input<float>(param++, "Scale");

    const VArray<float> &r_gon_sides = params.readonly_single_input<float>(param++, "R_gon Sides");
    const VArray<float> &r_gon_roundness = params.readonly_single_input<float>(param++,
                                                                               "R_gon Roundness");
    const VArray<float> &r_gon_exponent = params.readonly_single_input<float>(param++,
                                                                              "R_gon Exponent");
    const VArray<float> &sphere_exponent = params.readonly_single_input<float>(param++,
                                                                               "Sphere Exponent");

    MutableSpan<float> r_r_sphere_field = params.uninitialized_single_output_if_required<float>(
        param++, "R_sphere Field");
    MutableSpan<float> r_r_gon_parameter_field =
        params.uninitialized_single_output_if_required<float>(param++, "R_gon Parameter Field");
    MutableSpan<float> r_max_unit_parameter =
        params.uninitialized_single_output_if_required<float>(param++, "Max Unit Parameter");

    const bool calc_r_sphere_field = !r_r_sphere_field.is_empty();
    const bool calc_r_gon_parameter_field = !r_r_gon_parameter_field.is_empty();
    const bool calc_max_unit_parameter = !r_max_unit_parameter.is_empty();

    mask.foreach_index([&](const int64_t i) {
      float3 out_variables = calculate_out_fields_4d(
          calc_r_gon_parameter_field,
          calc_max_unit_parameter,
          normalize_r_gon_parameter_,
          elliptical_corners_,
          math::max(r_gon_sides[i], 2.0f),
          math::clamp(r_gon_roundness[i], 0.0f, 1.0f),
          math::max(r_gon_exponent[i], 0.0f),
          math::max(sphere_exponent[i], 0.0f),
          scale[i] * float4(coord[i].x, coord[i].y, coord[i].z, in_w[i]));

      if (calc_r_sphere_field) {
        r_r_sphere_field[i] = out_variables.x;
      }
      if (calc_r_gon_parameter_field) {
        r_r_gon_parameter_field[i] = out_variables.y;
      }
      if (calc_max_unit_parameter) {
        r_max_unit_parameter[i] = out_variables.z;
      }
    });
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    hints.allocates_array = false;
    hints.min_grain_size = 50;
    return hints;
  }
};

static void sh_node_raiko_base_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeTexRaikoBase &storage = node_storage(builder.node());
  builder.construct_and_set_matching_fn<RaikoBaseFunction>(storage.normalize_r_gon_parameter,
                                                           storage.elliptical_corners);
}

}  // namespace blender::nodes::node_shader_tex_raiko_base_cc

void register_node_type_sh_tex_raiko_base()
{
  namespace file_ns = blender::nodes::node_shader_tex_raiko_base_cc;

  static blender::bke::bNodeType ntype;

  sh_fn_node_type_base(&ntype, SH_NODE_TEX_RAIKO_BASE, "Raiko Base Texture", NODE_CLASS_TEXTURE);
  ntype.declare = file_ns::sh_node_tex_raiko_base_declare;
  ntype.draw_buttons = file_ns::node_shader_buts_tex_raiko_base;
  ntype.initfunc = file_ns::node_shader_init_tex_raiko_base;
  blender::bke::node_type_storage(
      &ntype, "NodeTexRaikoBase", node_free_standard_storage, node_copy_standard_storage);
  ntype.gpu_fn = file_ns::node_shader_gpu_tex_raiko_base;
  ntype.updatefunc = file_ns::node_shader_update_tex_raiko_base;
  ntype.build_multi_function = file_ns::sh_node_raiko_base_build_multi_function;

  blender::bke::node_register_type(&ntype);
}
