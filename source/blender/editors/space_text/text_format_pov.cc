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

/* -------------------------------------------------------------------- */
/** \name Local Literal Definitions
 * \{ */

/** Language Directives */
static Array<StringRef> &text_format_pov_literals_keyword()
{
  static Array<StringRef> map{
      /* clang-format off */
    "append",
    "break",
    "case",
    "debug",
    "declare",
    "default",
    "deprecated",
    "else",
    "elseif",
    "end",
    "error",
    "fclose",
    "fopen",
    "for",
    "if",
    "ifdef",
    "ifndef",
    "include",
    "local",
    "macro",
    "patch",
    "persistent",
    "range",
    "read",
    "render",
    "statistics",
    "switch",
    "undef",
    "version",
    "warning",
    "while",
    "write",
      /* clang-format on */
  };
  return map;
}

/* POV-Ray Built-in Variables
 * list is from...
 * http://www.povray.org/documentation/view/3.7.0/212/
 */

/** Float Functions */
static Array<StringRef> &text_format_pov_literals_reserved()
{
  static Array<StringRef> map{
      /* clang-format on */
      "abs",
      "acos",
      "acosh",
      "albedo",
      "altitude",
      "angle",
      "asc",
      "asin",
      "asinh",
      "atan",
      "atan2",
      "atand",
      "atanh",
      "bitwise_and",
      "bitwise_or",
      "bitwise_xor",
      "blink",
      "ceil",
      "child",
      "clipped_by",
      "collect",
      "conserve_energy",
      "cos",
      "cosh",
      "crand",
      "defined",
      "degrees",
      "dimension_size",
      "dimensions",
      "direction",
      "div",
      "evaluate",
      "exp",
      "file_exists",
      "floor",
      "form",
      "function",
      "gts_load",
      "gts_save",
      "inside",
      "int",
      "inverse",
      "jitter",
      "ln",
      "load_file",
      "location",
      "log",
      "look_at",
      "matrix",
      "max",
      "max_intersections",
      "max_trace",
      "metallic",
      "min",
      "mod",
      "phong_size",
      "pow",
      "precompute",
      "prod",
      "pwr",
      "quaternion",
      "radians",
      "rand",
      "reciprocal",
      "right",
      "rotate",
      "roughness",
      "save_file",
      "scale",
      "seed",
      "select",
      "shadowless",
      "sin",
      "sinh",
      "sky",
      "sqr",
      "sqrt",
      "strcmp",
      "strlen",
      "sturm",
      "sum",
      "tan",
      "tanh",
      "target",
      "tessel",
      "tesselate",
      "transform",
      "translate",
      "turb_depth",
      "up",
      "val",
      "vdot",
      "vlength",
      "warp",
      "with",

      /* Color Identifiers */
      "SRGB",
      "blue",
      "filter",
      "gamma",
      "gray",
      "green",
      "red",
      "rgb",
      "rgbf",
      "rgbft",
      "rgbt",
      "sRGB",
      "srgb",
      "srgbf",
      "srgbft",
      "srgbt",
      "transmit",
      /* Color Spaces */
      "hsl",
      "hsv",
      "pov",
      "xyl",
      "xyv",
      /* Vector Functions */
      "max_extent",
      "min_extent",
      "trace",
      "vaxis_rotate",
      "vcross",
      "vnormalize",
      "vrotate",
      "vturbulence",
      /* String Functions */
      "chr",
      "concat",
      "datetime",
      "file_time",
      "str",
      "strlwr",
      "strupr",
      "substr",
      "vstr",
      /* clang-format on */
  };
  return map;
}

/* POV-Ray Built-in Variables
 * list is from...
 * http://www.povray.org/documentation/view/3.7.0/212/
 */

/* Language Keywords */
static Array<StringRef> &text_format_pov_literals_builtins()
{
  static Array<StringRef> map{
      /* clang-format off */
    "aa_threshold",
    "absorption",
    "all",
    "all_intersections",
    "alpha",
    "arc_angle",
    "area_illumination",
    "array",
    "bend",
    "black_hole",
    "color_space",
    "colour_space",
    "component",
    "composite",
    "coords",
    "cube",
    "cutaway_textures",
    "dist_exp",
    "emission",
    "exterior",
    "falloff_angle",
    "file_gamma",
    "flatness",
    "flip",
    "hierarchy",
    "hypercomplex",
    "interior",
    "internal",
    "iridescence",
    "keep",
    "lommel_seeliger",
    "look_at",
    "major_radius",
    "material",
    "max_distance",
    "max_extent",
    "max_iteration",
    "metric",
    "minnaert",
    "move",
    "now",
    "offset",
    "oren_nayar",
    "orientation",
    "pattern",
    "photons",
    "planet",
    "pot",
    "precision",
    "radiosity",
    "reflection",
    "reflection_exponent",
    "refraction",
    "repeat",
    "roll",
    "scattering",
    "screw",
    "size",
    "slice",
    "smooth",
    "smooth_triangle",
    "solid",
    "subsurface",
    "texture",
    "tolerance",
    "toroidal",
    "type",
    "water_level",
    "width",

    /* Animation Options */
    "clock",
    "clock_delta",
    "clock_on",
    "final_clock",
    "final_frame",
    "frame_number",
    "global_settings",
    "image_height",
    "image_width",
    "initial_clock",
    "initial_frame",
    "input_file_name",

    /* Spline Identifiers */
    "akima_spline",
    "b_spline",
    "basic_x_spline",
    "bezier_spline",
    "conic_sweep",
    "cubic_spline",
    "extended_x_spline",
    "general_x_spline",
    "linear_spline",
    "linear_sweep",
    "natural_spline",
    "quadratic_spline",
    "sor_spline",
    "tcb_spline",

    /* Patterns */
    "agate",
    "aoi",
    "average",
    "binary",
    "boxed",
    "bozo",
    "brick",
    "bump_map",
    "bumps",
    "cells",
    "checker",
    "crackle",
    "cubic",
    "cylindrical",
    "density_file",
    "dents",
    "facets",
    "gradient",
    "granite",
    "hexagon",
    "image_map",
    "image_pattern",
    "julia",
    "leopard",
    "magnet",
    "mandel",
    "marble",
    "masonry",
    "onion",
    "pavement",
    "pigment_pattern",
    "planar",
    "proportion",
    "proximity",
    "quilted",
    "radial",
    "ripples",
    "slope",
    "spherical",
    "spiral1",
    "spiral2",
    "spotted",
    "square",
    "tile2",
    "tiles",
    "tiling",
    "triangular",
    "voronoi",
    "waves",
    "wood",
    "wrinkles",

    /* Objects */
    "ambient",
    "background",
    "bicubic_patch",
    "blob",
    "box",
    "brilliance",
    "camera",
    "color",
    "colour",
    "cone",
    "cubic",
    "cubic_spline",
    "cylinder",
    "difference",
    "diffuse",
    "disc",
    "displace",
    "fog",
    "galley",
    "height_field",
    "intermerge",
    "intersection",
    "interunion",
    "irid",
    "isosurface",
    "julia_fractal",
    "lathe",
    "lemon",
    "light_group",
    "light_source",
    "media",
    "merge",
    "mesh",
    "mesh2",
    "object",
    "ovus",
    "parametric",
    "phong",
    "plane",
    "poly",
    "polygon",
    "polynomial",
    "prism",
    "quadric",
    "quartic",
    "rainbow",
    "sky_sphere",
    "sor",
    "specular",
    "sphere",
    "sphere_sweep",
    "spline",
    "superellipsoid",
    "text",
    "torus",
    "triangle",
    "union",
    /* Built-in Vectors */
    "t",
    "u",
    "v",
    "x",
    "y",
    "z",
      /* clang-format on */
  };
  return map;
}
/**
 * POV modifiers.
 * See:
 * http://www.povray.org/documentation/view/3.7.0/212/
 */
static Array<StringRef> &text_format_pov_literals_specialvar()
{
  static Array<StringRef> map{
      /* clang-format off */
    "aa_level",
    "accuracy",
    "accuracy",
    "adaptive",
    "adc_bailout",
    "agate_turb",
    "albinos",
    "always_sample",
    "ambient_light",
    "amount",
    "assumed_gamma",
    "autostop",
    "bounded_by",
    "brick_size",
    "brightness",
    "bump_size",
    "caustics",
    "charset",
    "circular",
    "color_map",
    "colour_map",
    "contained_by",
    "control0",
    "control1",
    "count",
    "cubic",
    "cubic_wave",
    "density",
    "density_map",
    "dispersion",
    "dispersion_samples",
    "distance",
    "double_illuminate",
    "eccentricity",
    "error_bound",
    "expand_thresholds",
    "exponent",
    "extinction",
    "face_indices",
    "fade_color",
    "fade_colour",
    "fade_distance",
    "fade_power",
    "finish",
    "fixed",
    "fog_alt",
    "fog_offset",
    "fog_type",
    "frequency",
    "fresnel",
    "gather",
    "global_lights",
    "gray_threshold",
    "hf_gray_16",
    "hollow",
    "importance",
    "inbound",
    "inner",
    "inside_point",
    "inside_vector",
    "interior_texture",
    "interpolate",
    "intervals",
    "ior",
    "irid_wavelength",
    "lambda",
    "low_error_factor",
    "map_type",
    "material_map",
    "max_gradient",
    "max_sample",
    "max_trace_level",
    "maximal",
    "maximum_reuse",
    "media_attenuation",
    "media_interaction",
    "method",
    "minimal",
    "minimum_reuse",
    "mm_per_unit",
    "modulation",
    "mortar",
    "nearest_count",
    "no_bump_scale",
    "no_cache",
    "no_image",
    "no_radiosity",
    "no_reflection",
    "no_shadow",
    "noise_generator",
    "normal",
    "normal_indices",
    "normal_map",
    "normal_vectors",
    "number_of_waves",
    "octaves",
    "offset",
    "omega",
    "once",
    "open",
    "orient",
    "origin",
    "original",
    "outbound",
    "outside",
    "pass_through",
    "phase",
    "pigment",
    "pigment_map",
    "polarity",
    "poly_wave",
    "precision",
    "pretrace_end",
    "pretrace_start",
    "projected_through",
    "quick_color",
    "quick_colour",
    "ramp_wave",
    "ratio",
    "recursion_limit",
    "samples",
    "scallop_wave",
    "sine_wave",
    "slope_map",
    "spacing",
    "split_union",
    "strength",
    "texture_list",
    "texture_map",
    "thickness",
    "threshold",
    "translucency",
    "triangle_wave",
    "turbulence",
    "u_steps",
    "use_alpha",
    "use_color",
    "use_colour",
    "use_index",
    "uv_indices",
    "uv_mapping",
    "uv_vectors",
    "v_steps",
    "vertex_vectors",

    /* Light Types and options. */
    "area_light",
    "fade_power",
    "falloff",
    "looks_like",
    "parallel",
    "point_at",
    "radius",
    "spotlight",
    "tightness",

    /* Camera Types and options. */
    "aitoff_hammer",
    "aperture",
    "balthasart",
    "behrmann",
    "blur_samples",
    "camera_direction",
    "camera_location",
    "camera_right",
    "camera_type",
    "camera_up",
    "confidence",
    "eckert_iv",
    "eckert_vi",
    "edwards",
    "fisheye",
    "focal_point",
    "gall",
    "hobo_dyer",
    "icosa",
    "lambert_azimuthal",
    "lambert_cylindrical",
    "mercator",
    "mesh_camera",
    "miller_cylindrical",
    "mollweide",
    "octa",
    "omni_directional_stereo",
    "omnimax",
    "orthographic",
    "panoramic",
    "parallaxe",
    "perspective",
    "peters",
    "plate_carree",
    "smyth_craster",
    "stereo",
    "tetra",
    "ultra_wide_angle",
    "van_der_grinten",
    "variance",
      /* clang-format on */
  };
  return map;
}

/** POV Built-in Constants. */
static Array<StringRef> &text_format_pov_literals_bool()
{
  static Array<StringRef> map{
      /* clang-format off */
    "false",
    "no",
    "off",
    "on",
    "pi",
    "tau",
    "true",
    "unofficial",
    "yes",
    /* Encodings. */
    "ascii",
    "bt2020",
    "bt709",
    "sint16be",
    "sint16le",
    "sint32be",
    "sint32le",
    "sint8",
    "uint16be",
    "uint16le",
    "uint8",
    "utf8",
    /* File-types. */
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
    "tiff",
    "ttf",
      /* clang-format on */
  };
  return map;
}
/** \} */

/* -------------------------------------------------------------------- */
/** \name Local Functions (for #TextFormatType::format_line)
 * \{ */

/**
 * POV keyword (minus boolean & 'nil').
 * See:
 * http://www.povray.org/documentation/view/3.7.0/212/
 */

static int txtfmt_pov_find_keyword(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_pov_literals_keyword(), string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_pov_find_reserved_keywords(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_pov_literals_reserved(), string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_pov_find_reserved_builtins(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_pov_literals_builtins(), string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_pov_find_specialvar(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_pov_literals_specialvar(), string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_pov_find_bool(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_pov_literals_bool(), string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "Nonetheless") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static char txtfmt_pov_format_identifier(const char *str)
{
  char fmt;

  /* Keep aligned args for readability. */
  /* clang-format off */

  if        (txtfmt_pov_find_specialvar(str)        != -1) { fmt = FMT_TYPE_SPECIAL;
  } else if (txtfmt_pov_find_keyword(str)           != -1) { fmt = FMT_TYPE_KEYWORD;
  } else if (txtfmt_pov_find_reserved_keywords(str) != -1) { fmt = FMT_TYPE_RESERVED;
  } else if (txtfmt_pov_find_reserved_builtins(str) != -1) { fmt = FMT_TYPE_DIRECTIVE;
  } else                                                   { fmt = FMT_TYPE_DEFAULT;
  }

  /* clang-format on */

  return fmt;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Format Line Implementation (#TextFormatType::format_line)
 * \{ */

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
      else if (prev != FMT_TYPE_DEFAULT && (i = txtfmt_pov_find_bool(str)) != -1) {
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
        if        ((i = txtfmt_pov_find_specialvar(str))        != -1) { prev = FMT_TYPE_SPECIAL;
        } else if ((i = txtfmt_pov_find_keyword(str))           != -1) { prev = FMT_TYPE_KEYWORD;
        } else if ((i = txtfmt_pov_find_reserved_keywords(str)) != -1) { prev = FMT_TYPE_RESERVED;
        } else if ((i = txtfmt_pov_find_reserved_builtins(str)) != -1) { prev = FMT_TYPE_DIRECTIVE;
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

/** \} */

/* -------------------------------------------------------------------- */
/** \name Registration
 * \{ */

void ED_text_format_register_pov()
{
  static TextFormatType tft = {nullptr};
  static const char *ext[] = {"pov", "inc", "mcr", "mac", nullptr};

  tft.format_identifier = txtfmt_pov_format_identifier;
  tft.format_line = txtfmt_pov_format_line;
  tft.ext = ext;
  tft.comment_line = "//";

  ED_text_format_register(&tft);

  text_format_string_literals_sort_for_lookup(text_format_pov_literals_keyword());
  text_format_string_literals_sort_for_lookup(text_format_pov_literals_reserved());
  text_format_string_literals_sort_for_lookup(text_format_pov_literals_builtins());
  text_format_string_literals_sort_for_lookup(text_format_pov_literals_specialvar());
  text_format_string_literals_sort_for_lookup(text_format_pov_literals_bool());
}

/** \} */
