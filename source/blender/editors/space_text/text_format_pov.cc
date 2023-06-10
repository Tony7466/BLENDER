/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sptext
 */

#include <cstring>

#include "BLI_blenlib.h"

#include "DNA_space_types.h"
#include "DNA_text_types.h"

#include "BKE_text.h"

#include "text_format.hh"

/* *** POV Keywords (for format_line) *** */

/**
 * POV keyword (minus boolean & 'nil').
 * See:
 * http://www.povray.org/documentation/view/3.7.0/212/
 */

/* Language Directives */
/* clang-format off */
static Array<StringRef> text_format_pov_keyword_literals = {
    "deprecated",
    "persistent",
    "statistics",
    "version",
    "warning",
    "declare",
    "default",
    "include",
    "append",
    "elseif",
    "debug",
    "break",
    "else",
    "error",
    "fclose",
    "fopen",
    "ifndef",
    "ifdef",
    "patch",
    "local",
    "macro",
    "range",
    "read",
    "render",
    "switch",
    "undef",
    "while",
    "write",
    "case",
    "end",
    "for",
    "if",
};
/* clang-format on */
/* POV-Ray Built-in Variables
 * list is from...
 * http://www.povray.org/documentation/view/3.7.0/212/
 */

/* Float Functions */
/* clang-format on */
static Array<StringRef> text_format_pov_reserved_literals{
    "conserve_energy",
    "max_intersections",
    "dimension_size",
    "bitwise_and",
    "bitwise_or",
    "bitwise_xor",
    "file_exists",
    "precompute",
    "dimensions",
    "clipped_by",
    "shadowless",
    "turb_depth",
    "reciprocal",
    "quaternion",
    "phong_size",
    "tesselate",
    "save_file",
    "load_file",
    "max_trace",
    "transform",
    "translate",
    "direction",
    "roughness",
    "metallic",
    "gts_load",
    "gts_save",
    "location",
    "altitude",
    "function",
    "evaluate",
    "inverse",
    "collect",
    "target",
    "albedo",
    "rotate",
    "matrix",
    "look_at",
    "jitter",
    "angle",
    "right",
    "scale",
    "child",
    "crand",
    "blink",
    "defined",
    "degrees",
    "inside",
    "radians",
    "vlength",
    "select",
    "floor",
    "strcmp",
    "strlen",
    "tessel",
    "sturm",
    "abs",
    "acosh",
    "prod",
    "with",
    "acos",
    "asc",
    "asinh",
    "asin",
    "atan2",
    "atand",
    "atanh",
    "atan",
    "ceil",
    "warp",
    "cosh",
    "log",
    "max",
    "min",
    "mod",
    "pow",
    "rand",
    "seed",
    "form",
    "sinh",
    "sqrt",
    "tanh",
    "vdot",
    "sin",
    "sqr",
    "sum",
    "pwr",
    "tan",
    "val",
    "cos",
    "div",
    "exp",
    "int",
    "sky",
    "up",
    "ln",

    /* Color Identifiers */
    "transmit",
    "filter",
    "srgbft",
    "srgbf",
    "srgbt",
    "rgbft",
    "gamma",
    "green",
    "blue",
    "gray",
    "srgb",
    "sRGB",
    "SRGB",
    "rgbf",
    "rgbt",
    "rgb",
    "red",
    /* Color Spaces */
    "pov",
    "hsl",
    "hsv",
    "xyl",
    "xyv",
    /* Vector Functions */
    "vaxis_rotate",
    "vturbulence",
    "min_extent",
    "vnormalize",
    "max_extent",
    "vrotate",
    "vcross",
    "trace",
    /* String Functions */
    "file_time",
    "datetime",
    "concat",
    "strlwr",
    "strupr",
    "substr",
    "vstr",
    "chr",
    "str",
};
/* clang-format on */
/* POV-Ray Built-in Variables
 * list is from...
 * http://www.povray.org/documentation/view/3.7.0/212/
 */

/* Language Keywords */
/* clang-format off */
static Array<StringRef> text_format_pov_builtins_literals = {
    "reflection_exponent",
    "area_illumination",
    "all_intersections",
    "cutaway_textures",
    "smooth_triangle",
    "lommel_seeliger",
    "falloff_angle",
    "aa_threshold",
    "hypercomplex",
    "major_radius",
    "max_distance",
    "max_iteration",
    "colour_space",
    "color_space",
    "iridescence",
    "subsurface",
    "scattering",
    "absorption",
    "water_level",
    "reflection",
    "max_extent",
    "oren_nayar",
    "refraction",
    "hierarchy",
    "radiosity",
    "tolerance",
    "interior",
    "toroidal",
    "emission",
    "material",
    "internal",
    "photons",
    "arc_angle",
    "minnaert",
    "texture",
    "array",
    "black_hole",
    "component",
    "composite",
    "coords",
    "cube",
    "dist_exp",
    "exterior",
    "file_gamma",
    "flatness",
    "planet",
    "screw",
    "keep",
    "flip",
    "move",
    "roll",
    "look_at",
    "metric",
    "offset",
    "orientation",
    "pattern",
    "precision",
    "width",
    "repeat",
    "bend",
    "size",
    "alpha",
    "slice",
    "smooth",
    "solid",
    "all",
    "now",
    "pot",
    "type",

    /* Animation Options */
    "global_settings",
    "input_file_name",
    "initial_clock",
    "initial_frame",
    "frame_number",
    "image_height",
    "image_width",
    "final_clock",
    "final_frame",
    "clock_delta",
    "clock_on",
    "clock",

    /* Spline Identifiers */
    "extended_x_spline",
    "general_x_spline",
    "quadratic_spline",
    "basic_x_spline",
    "natural_spline",
    "linear_spline",
    "bezier_spline",
    "akima_spline",
    "cubic_spline",
    "sor_spline",
    "tcb_spline",
    "linear_sweep",
    "conic_sweep",
    "b_spline",

    /* Patterns */
    "pigment_pattern",
    "image_pattern",
    "density_file",
    "cylindrical",
    "proportion",
    "triangular",
    "image_map",
    "proximity",
    "spherical",
    "bump_map",
    "wrinkles",
    "average",
    "voronoi",
    "masonry",
    "binary",
    "boxed",
    "bozo",
    "brick",
    "bumps",
    "cells",
    "checker",
    "crackle",
    "cubic",
    "dents",
    "facets",
    "gradient",
    "granite",
    "hexagon",
    "julia",
    "leopard",
    "magnet",
    "mandel",
    "marble",
    "onion",
    "pavement",
    "planar",
    "quilted",
    "radial",
    "ripples",
    "slope",
    "spiral1",
    "spiral2",
    "spotted",
    "square",
    "tile2",
    "tiling",
    "tiles",
    "waves",
    "wood",
    "agate",
    "aoi",

    /* Objects */
    "superellipsoid",
    "bicubic_patch",
    "julia_fractal",
    "height_field",
    "cubic_spline",
    "sphere_sweep",
    "light_group",
    "light_source",
    "intersection",
    "isosurface",
    "background",
    "sky_sphere",
    "cylinder",
    "difference",
    "brilliance",
    "parametric",
    "interunion",
    "intermerge",
    "polynomial",
    "displace",
    "specular",
    "ambient",
    "diffuse",
    "polygon",
    "quadric",
    "quartic",
    "rainbow",
    "sphere",
    "spline",
    "prism",
    "camera",
    "galley",
    "cubic",
    "phong",
    "cone",
    "blob",
    "box",
    "disc",
    "fog",
    "lathe",
    "merge",
    "mesh2",
    "mesh",
    "object",
    "ovus",
    "lemon",
    "plane",
    "poly",
    "irid",
    "sor",
    "text",
    "torus",
    "triangle",
    "union",
    "colour",
    "color",
    "media",
    /* Built-in Vectors */
    "t",
    "u",
    "v",
    "x",
    "y",
    "z",
};
/* clang-format on */
/**
 * POV modifiers.
 * See:
 * http://www.povray.org/documentation/view/3.7.0/212/
 */
/* clang-format off */
static Array<StringRef> text_format_pov_specialvar_literals = {
    "dispersion_samples",
    "projected_through",
    "double_illuminate",
    "expand_thresholds",
    "media_interaction",
    "media_attenuation",
    "low_error_factor",
    "recursion_limit",
    "interior_texture",
    "max_trace_level",
    "gray_threshold",
    "pretrace_start",
    "normal_indices",
    "normal_vectors",
    "vertex_vectors",
    "noise_generator",
    "irid_wavelength",
    "number_of_waves",
    "ambient_light",
    "inside_vector",
    "face_indices",
    "texture_list",
    "max_gradient",
    "uv_indices",
    "uv_vectors",
    "fade_distance",
    "global_lights",
    "no_bump_scale",
    "pretrace_end",
    "no_radiosity",
    "no_reflection",
    "assumed_gamma",
    "scallop_wave",
    "triangle_wave",
    "nearest_count",
    "maximum_reuse",
    "minimum_reuse",
    "always_sample",
    "translucency",
    "eccentricity",
    "contained_by",
    "inside_point",
    "adc_bailout",
    "density_map",
    "split_union",
    "mm_per_unit",
    "agate_turb",
    "bounded_by",
    "brick_size",
    "hf_gray_16",
    "dispersion",
    "extinction",
    "thickness",
    "color_map",
    "colour_map",
    "cubic_wave",
    "fade_colour",
    "fade_power",
    "fade_color",
    "normal_map",
    "pigment_map",
    "quick_color",
    "quick_colour",
    "material_map",
    "pass_through",
    "interpolate",
    "texture_map",
    "error_bound",
    "brightness",
    "use_color",
    "use_alpha",
    "use_colour",
    "use_index",
    "uv_mapping",
    "importance",
    "max_sample",
    "intervals",
    "sine_wave",
    "slope_map",
    "poly_wave",
    "no_shadow",
    "ramp_wave",
    "precision",
    "original",
    "accuracy",
    "map_type",
    "no_image",
    "distance",
    "autostop",
    "caustics",

    "octaves",
    "aa_level",
    "frequency",
    "fog_offset",
    "modulation",
    "outbound",
    "no_cache",
    "pigment",
    "charset",
    "inbound",
    "outside",
    "inner",
    "turbulence",
    "threshold",
    "accuracy",
    "polarity",
    "bump_size",
    "circular",
    "control0",
    "control1",
    "maximal",
    "minimal",
    "fog_type",
    "fog_alt",
    "samples",
    "origin",
    "amount",
    "adaptive",
    "exponent",
    "strength",
    "density",
    "fresnel",
    "albinos",
    "finish",
    "method",
    "omega",
    "fixed",
    "spacing",
    "u_steps",
    "v_steps",
    "offset",
    "hollow",
    "gather",
    "lambda",
    "mortar",
    "cubic",
    "count",
    "once",
    "orient",
    "normal",
    "phase",
    "ratio",
    "open",
    "ior",

    /* Light Types and options. */
    "area_light",
    "looks_like",
    "fade_power",
    "tightness",
    "spotlight",
    "parallel",
    "point_at",
    "falloff",
    "radius",

    /* Camera Types and options. */
    "omni_directional_stereo",
    "lambert_cylindrical",
    "miller_cylindrical",
    "lambert_azimuthal",
    "ultra_wide_angle",
    "camera_direction",
    "camera_location",
    "van_der_grinten",
    "aitoff_hammer",
    "smyth_craster",
    "orthographic",
    "camera_right",
    "blur_samples",
    "plate_carree",
    "camera_type",
    "perspective",
    "mesh_camera",
    "focal_point",
    "balthasart",
    "confidence",
    "parallaxe",
    "hobo_dyer",
    "camera_up",
    "panoramic",
    "eckert_vi",
    "eckert_iv",
    "mollweide",
    "aperture",
    "behrmann",
    "variance",
    "stereo",
    "icosa",
    "tetra",
    "octa",
    "mercator",
    "omnimax",
    "fisheye",
    "edwards",
    "peters",
    "gall",
};
/* clang-format on */
/* POV Built-in Constants. */
/* clang-format off */
static Array<StringRef> text_format_pov_bool_literals = {
    "unofficial",
    "false",
    "no",
    "off",
    "true",
    "yes",
    "on",
    "pi",
    "tau",
    /* Encodings. */
    "sint16be",
    "sint16le",
    "sint32be",
    "sint32le",
    "uint16be",
    "uint16le",
    "bt2020",
    "bt709",
    "sint8",
    "uint8",
    "ascii",
    "utf8",
    /* File-types. */
    "tiff",
    "df3",
    "exr",
    "gif",
    "hdr",
    "iff",
    "jpeg",
    "pgm",
    "png",
    "ppm",
    "sys",
    "tga",
    "ttf",
};
/* clang-format on */
static char txtfmt_pov_format_identifier(const char *str)
{
  char fmt;

  /* Keep aligned args for readability. */
  /* clang-format off */

  if        (find_keyword_length(text_format_pov_specialvar_literals,str)        != -1) { fmt = FMT_TYPE_SPECIAL;
  } else if (find_keyword_length(text_format_pov_keyword_literals,str)           != -1) { fmt = FMT_TYPE_KEYWORD;
  } else if (find_keyword_length(text_format_pov_reserved_literals,str) != -1) { fmt = FMT_TYPE_RESERVED;
  } else if (find_keyword_length(text_format_pov_builtins_literals,str) != -1) { fmt = FMT_TYPE_DIRECTIVE;
  } else                                                   { fmt = FMT_TYPE_DEFAULT;
  }

  /* clang-format on */

  return fmt;
}

static void txtfmt_pov_format_line(SpaceText *st, TextLine *line, const bool do_next)
{
  FlattenString fs;
  const char *str;
  char *fmt;
  char cont_orig, cont, find, prev = ' ';
  int len, i;

  /* Get continuation from previous line */
  if (line->prev && line->prev->format != nullptr) {
    fmt = line->prev->format;
    cont = fmt[strlen(fmt) + 1]; /* Just after the null-terminator */
    BLI_assert((FMT_CONT_ALL & cont) == cont);
  }
  else {
    cont = FMT_CONT_NOP;
  }

  /* Get original continuation from this line */
  if (line->format != nullptr) {
    fmt = line->format;
    cont_orig = fmt[strlen(fmt) + 1]; /* Just after the null-terminator */
    BLI_assert((FMT_CONT_ALL & cont_orig) == cont_orig);
  }
  else {
    cont_orig = 0xFF;
  }

  len = flatten_string(st, &fs, line->line);
  str = fs.buf;
  if (!text_check_format_len(line, len)) {
    flatten_string_free(&fs);
    return;
  }
  fmt = line->format;

  while (*str) {
    /* Handle escape sequences by skipping both \ and next char */
    if (*str == '\\') {
      *fmt = prev;
      fmt++;
      str++;
      if (*str == '\0') {
        break;
      }
      *fmt = prev;
      fmt++;
      str += BLI_str_utf8_size_safe(str);
      continue;
    }
    /* Handle continuations */
    if (cont) {
      /* C-Style comments */
      if (cont & FMT_CONT_COMMENT_C) {
        if (*str == '*' && *(str + 1) == '/') {
          *fmt = FMT_TYPE_COMMENT;
          fmt++;
          str++;
          *fmt = FMT_TYPE_COMMENT;
          cont = FMT_CONT_NOP;
        }
        else {
          *fmt = FMT_TYPE_COMMENT;
        }
        /* Handle other comments */
      }
      else {
        find = (cont & FMT_CONT_QUOTEDOUBLE) ? '"' : '\'';
        if (*str == find) {
          cont = 0;
        }
        *fmt = FMT_TYPE_STRING;
      }

      str += BLI_str_utf8_size_safe(str) - 1;
    }
    /* Not in a string... */
    else {
      /* C-Style (multi-line) comments */
      if (*str == '/' && *(str + 1) == '*') {
        cont = FMT_CONT_COMMENT_C;
        *fmt = FMT_TYPE_COMMENT;
        fmt++;
        str++;
        *fmt = FMT_TYPE_COMMENT;
      }
      /* Single line comment */
      else if (*str == '/' && *(str + 1) == '/') {
        text_format_fill(&str, &fmt, FMT_TYPE_COMMENT, len - int(fmt - line->format));
      }
      else if (ELEM(*str, '"', '\'')) {
        /* Strings */
        find = *str;
        cont = (*str == '"') ? FMT_CONT_QUOTEDOUBLE : FMT_CONT_QUOTESINGLE;
        *fmt = FMT_TYPE_STRING;
      }
      /* White-space (all white-space has been converted to spaces). */
      else if (*str == ' ') {
        *fmt = FMT_TYPE_WHITESPACE;
      }
      /* Numbers (digits not part of an identifier and periods followed by digits) */
      else if ((prev != FMT_TYPE_DEFAULT && text_check_digit(*str)) ||
               (*str == '.' && text_check_digit(*(str + 1))))
      {
        *fmt = FMT_TYPE_NUMERAL;
      }
      /* Booleans */
      else if (prev != FMT_TYPE_DEFAULT &&
               (i = find_keyword_length(text_format_pov_bool_literals, str)) != -1)
      {
        if (i > 0) {
          text_format_fill_ascii(&str, &fmt, FMT_TYPE_NUMERAL, i);
        }
        else {
          str += BLI_str_utf8_size_safe(str) - 1;
          *fmt = FMT_TYPE_DEFAULT;
        }
      }
      /* Punctuation */
      else if (text_check_delim(*str)) {
        *fmt = FMT_TYPE_SYMBOL;
      }
      /* Identifiers and other text (no previous white-space/delimiters so text continues). */
      else if (prev == FMT_TYPE_DEFAULT) {
        str += BLI_str_utf8_size_safe(str) - 1;
        *fmt = FMT_TYPE_DEFAULT;
      }
      /* Not white-space, a digit, punctuation, or continuing text.
       * Must be new, check for special words. */
      else {
        /* Keep aligned arguments for readability. */
        /* clang-format off */

        /* Special vars(v) or built-in keywords(b) */
        /* keep in sync with `txtfmt_pov_format_identifier()`. */
        if        ((i = find_keyword_length(text_format_pov_specialvar_literals,str))        != -1) { prev = FMT_TYPE_SPECIAL;
        } else if ((i = find_keyword_length(text_format_pov_keyword_literals,str))           != -1) { prev = FMT_TYPE_KEYWORD;
        } else if ((i = find_keyword_length(text_format_pov_reserved_literals,str)) != -1) { prev = FMT_TYPE_RESERVED;
        } else if ((i = find_keyword_length(text_format_pov_builtins_literals,str)) != -1) { prev = FMT_TYPE_DIRECTIVE;
        }

        /* clang-format on */

        if (i > 0) {
          text_format_fill_ascii(&str, &fmt, prev, i);
        }
        else {
          str += BLI_str_utf8_size_safe(str) - 1;
          *fmt = FMT_TYPE_DEFAULT;
        }
      }
    }
    prev = *fmt;
    fmt++;
    str++;
  }

  /* Terminate and add continuation char */
  *fmt = '\0';
  fmt++;
  *fmt = cont;

  /* If continuation has changed and we're allowed, process the next line */
  if (cont != cont_orig && do_next && line->next) {
    txtfmt_pov_format_line(st, line->next, do_next);
  }

  flatten_string_free(&fs);
}

void ED_text_format_register_pov()
{
  static TextFormatType tft = {nullptr};
  static const char *ext[] = {"pov", "inc", "mcr", "mac", nullptr};

  tft.format_identifier = txtfmt_pov_format_identifier;
  tft.format_line = txtfmt_pov_format_line;
  tft.ext = ext;
  tft.comment_line = "//";

  ED_text_format_register(&tft);

  sort_string_literals(text_format_pov_keyword_literals);
  sort_string_literals(text_format_pov_reserved_literals);
  sort_string_literals(text_format_pov_builtins_literals);
  sort_string_literals(text_format_pov_specialvar_literals);
  sort_string_literals(text_format_pov_bool_literals);
}
