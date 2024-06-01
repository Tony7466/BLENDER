/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Signed distance to rounded box, centered at origin.
 * Reference: https://iquilezles.org/articles/distfunctions2d/ */
float sdfRoundedBox(vec2 pos, vec2 size, float radius)
{
  vec2 q = abs(pos) - size + radius;
  return min(max(q.x,q.y),0.0) + length(max(q,0.0)) - radius;
}

vec4 color_unpack(uint rgba)
{
  return vec4(rgba & 0xFF, (rgba >> 8) & 0xFF, (rgba >> 16) & 0xFF, rgba >> 24) * (1.0 / 255.0);
}

vec3 color_shade(vec3 rgb, float shade)
{
  rgb += vec3(shade/255.0);
  rgb = clamp(rgb, vec3(0.0), vec3(1.0));
  return rgb;
}

vec4 add_outline(float d, float extra_half_width, float inset, vec4 cur, vec4 outline_color)
{
    float f = abs(d + inset) - extra_half_width;
    float a = clamp(1.0 - f, 0.0, 1.0);
    return mix(cur, outline_color, a);
}

//@TODO: premultiplied alpha?
vec4 blend_color(vec4 cur, vec4 color)
{
  if (cur.a == 0)
    cur = color;
  else
    cur = mix(cur, color, color.a);
  return cur;
}

void main()
{
  vec2 uv = uvInterp;
  vec2 co = coInterp;

  vec2 viewToPixels = vec2(1.0/context_data.pixelx, 1.0/context_data.pixely);

  SeqStripDrawData strip = strip_data[strip_id];
  float bleft = strip.left_handle;
  float bright = strip.right_handle;
  float btop = strip.bottom;
  float bbottom = strip.top;

  vec2 bsize = vec2(bright-bleft, bbottom-btop) * 0.5;
  vec2 bcenter = vec2(bright+bleft, bbottom+btop) * 0.5;
        
  bsize *= viewToPixels;
  bcenter *= viewToPixels;
  vec2 pxy = co * viewToPixels;

  float radius = context_data.round_radius;
  radius = min(radius,min(bsize.x,bsize.y));
    
  float d = sdfRoundedBox(pxy - bcenter, bsize, radius);

  vec4 col = vec4(0.0);

  bool bottom_part = (strip.flags & GPU_SEQ_FLAG_BOTTOM_PART) != 0;

  if (bottom_part) {

    col = color_unpack(strip.col_background);
    /* Darker background for multi-image strip hold still regions. */
    if ((strip.flags & GPU_SEQ_FLAG_SINGLE_IMAGE) == 0) {
      if (co.x < strip.content_start || co.x > strip.content_end) {
        col.rgb = color_shade(col.rgb, -35.0);
      }
    }

    /* Color band. */
    if ((strip.flags & GPU_SEQ_FLAG_COLOR_BAND) != 0) {
      if (co.y < strip.strip_content_top) {
        col.rgb = color_unpack(strip.col_color_band).rgb;
        /* Darker line to better separate the color band. */
        if (co.y > strip.strip_content_top - context_data.pixely) {
          col.rgb = color_shade(col.rgb, -20.0);
        }
      }
    }

    /* Transition. */
    if ((strip.flags & GPU_SEQ_FLAG_TRANSITION) != 0) {
      if (co.x >= strip.content_start && co.x <= strip.content_end && co.y < strip.strip_content_top) {
        float diag_y = strip.strip_content_top - (strip.strip_content_top - strip.bottom) * (co.x - strip.content_start) / (strip.content_end - strip.content_start);
        uint transition_color = co.y <= diag_y ? strip.col_transition_in : strip.col_transition_out;
        col.rgb = color_unpack(transition_color).rgb;
      }
    }
  }
  else {
    /* Missing media. */
    if ((strip.flags & GPU_SEQ_FLAG_MISSING_TITLE) != 0) {
      if (co.y > strip.strip_content_top) {
        col = vec4(112.0/255.0, 0.0, 0.0, 230.0/255.0);
      }
    }
    if ((strip.flags & GPU_SEQ_FLAG_MISSING_CONTENT) != 0) {
      if (co.y <= strip.strip_content_top) {
        col = vec4(64.0/255.0, 0.0, 0.0, 230.0/255.0);
      }
    }

    /* Locked. */
    if ((strip.flags & GPU_SEQ_FLAG_LOCKED) != 0) {
      if (co.y <= strip.strip_content_top) {
        float phase = mod(gl_FragCoord.x + gl_FragCoord.y, 12.0);
        if (phase >= 8.0) {
          col = blend_color(col, vec4(0.0, 0.0, 0.0, 0.25));
        }
      }
    }

    /* Highlight. */
    if ((strip.flags & GPU_SEQ_FLAG_HIGHLIGHT) != 0) {
      col = blend_color(col, vec4(1.0, 1.0, 1.0, 48.0/255.0));
    }

    /* Handles. */
    if ((strip.flags & GPU_SEQ_FLAG_HANDLES) != 0) {
      if (co.x >= strip.left_handle && co.x < strip.left_handle + strip.handle_width) {
        col = blend_color(col, color_unpack(strip.col_handle_left));
      }
      if (co.x > strip.right_handle - strip.handle_width && co.x <= strip.right_handle) {
        col = blend_color(col, color_unpack(strip.col_handle_right));
      }
    }
  }

  /* Outside of strip rounded rect? */
  if (d > 0.0) {
    col = vec4(0.0);
  }

  /* Outline. */
  if (!bottom_part) {
    bool selected = (strip.flags & GPU_SEQ_FLAG_SELECTED) != 0;
    vec4 col_outline = color_unpack(strip.col_outline);
    if (selected) {
      /* Inset 1px line with backround color. */
      col = add_outline(d, 0.0, 1.0, col, color_unpack(context_data.col_back));
      /* 2x wide outline. */
      col = add_outline(d, 0.5, -0.5, col, col_outline);
    }
    else {
      col = add_outline(d, 0.0, 0.0, col, col_outline);
    }
  }

  fragColor = col;
}
