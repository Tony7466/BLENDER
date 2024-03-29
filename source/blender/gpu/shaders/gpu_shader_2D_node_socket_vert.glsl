/* SPDX-FileCopyrightText: 2018-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Values in `eNodeSocketDisplayShape` in DNA_node_types.h. Keep in sync. */
#define SOCK_DISPLAY_SHAPE_CIRCLE 0
#define SOCK_DISPLAY_SHAPE_SQUARE 1
#define SOCK_DISPLAY_SHAPE_DIAMOND 2
#define SOCK_DISPLAY_SHAPE_CIRCLE_DOT 3
#define SOCK_DISPLAY_SHAPE_SQUARE_DOT 4
#define SOCK_DISPLAY_SHAPE_DIAMOND_DOT 5

#define rect parameters[widgetID * MAX_PARAM + 0]
#define colorInner parameters[widgetID * MAX_PARAM + 1]
#define colorOutline parameters[widgetID * MAX_PARAM + 2]
#define outlineThickness parameters[widgetID * MAX_PARAM + 3].x
#define outlineOffset parameters[widgetID * MAX_PARAM + 3].y
#define dotRadius parameters[widgetID * MAX_PARAM + 3].z
#define shape parameters[widgetID * MAX_PARAM + 3].w

#define AA_SIZE 0.75
#define IS_DIAMOND \
  (shapeFlags == SOCK_DISPLAY_SHAPE_DIAMOND || shapeFlags == SOCK_DISPLAY_SHAPE_DIAMOND_DOT)
#define HAS_DOT \
  (shapeFlags > 2)

/* Offsetting by a pixel further to avoid losing pixels. */
vec2 ofs = vec2(outlineOffset + 1.0, -outlineOffset - 1.0);

/* Calculate size of the original rectangle before expanding it based on the offset for the outline
 */
vec2 rectSize = rect.yw - rect.xz;
float minSize = min(rectSize.x, rectSize.y);

/* Set the parameters for the sdf function that is used to draw the socket. */
#define CIRCLE_RADIUS 0.5
#define SQUARE_RADIUS 0.5
#define DIAMOND_RADIUS 0.4

vec3 shapeRadii = vec3(CIRCLE_RADIUS, SQUARE_RADIUS, DIAMOND_RADIUS);
vec3 cornerRoundness = vec3(1.0, 0.4, 0.4);

int shapeFlags = int(shape);
int shapeIndex = shapeFlags % 3;
float shapeRadius = shapeRadii[shapeIndex];
float cornerRadius = shapeRadius * cornerRoundness[shapeIndex];

float pow2(float x)
{
  return x * x;
}

void main()
{
  vec2 pos;
  switch (gl_VertexID) {
    default:
    case 0: {
      pos = rect.xz + ofs.yy;
      break;
    }
    case 1: {
      pos = rect.xw + ofs.yx;
      break;
    }
    case 2: {
      pos = rect.yz + ofs.xy;
      break;
    }
    case 3: {
      pos = rect.yw + ofs.xx;
      break;
    }
  }

  gl_Position = ModelViewProjectionMatrix * vec4(pos, 0.0, 1.0);

  vec2 centeredCoordinates = pos - ((rect.xz + rect.yw) / 2.0);
  uv = centeredCoordinates / minSize;

  /* Calculate the necessary "extrusion" of the coordinates to draw the middle part of
   * multi sockets. */
  float aspect = rectSize.x / rectSize.y;
  extrusion = (aspect > 1.0) ? vec2((aspect - 1.0) / 2.0, 0.0) :
                               vec2(0.0, ((1.0 / aspect) - 1.0) / 2.0);

  /* Thresholds for the masks in UV Space. Use squared values, so we can use the squared length in
   * the fragment shader. */
  float InnerOutlineUVSquared1 = pow2(
      ((outlineOffset - 0.5 * outlineThickness - AA_SIZE) / minSize) + cornerRadius);
  float InnerOutlineUVSquared2 = pow2(((outlineOffset - 0.5 * outlineThickness) / minSize) +
                                      cornerRadius);
  float OuterOutlineUVSquared1 = pow2(
      ((outlineOffset + 0.5 * outlineThickness - AA_SIZE) / minSize) + cornerRadius);
  float OuterOutlineUVSquared2 = pow2(((outlineOffset + 0.5 * outlineThickness) / minSize) +
                                      cornerRadius);

  thresholds = vec4(InnerOutlineUVSquared1,
                    InnerOutlineUVSquared2,
                    OuterOutlineUVSquared1,
                    OuterOutlineUVSquared2);

  /* Thresholds for the mask of the dot. */
  bool has_dot = HAS_DOT;
  float dotRadiusSquared1 = has_dot ? pow2((dotRadius - AA_SIZE) / minSize) : (-1.0f);
  float dotRadiusSquared2 = has_dot ? pow2(dotRadius / minSize) : 0.0f;

  dotThresholds = vec2(dotRadiusSquared1, dotRadiusSquared2);

  /* Shape parameters. */
  sdf_shape_radius = shapeRadius - cornerRadius;
  is_diamond = IS_DIAMOND ? 1 : 0;

  /* Pass through parameters. */
  finalColor = colorInner;
  finalOutlineColor = colorOutline;
}
