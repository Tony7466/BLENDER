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
static StringRef text_format_pov_literals_keyword[]{
    /* clang-format off */
    StringRef("append"),
    StringRef("break"),
    StringRef("case"),
    StringRef("debug"),
    StringRef("declare"),
    StringRef("default"),
    StringRef("deprecated"),
    StringRef("else"),
    StringRef("elseif"),
    StringRef("end"),
    StringRef("error"),
    StringRef("fclose"),
    StringRef("fopen"),
    StringRef("for"),
    StringRef("if"),
    StringRef("ifdef"),
    StringRef("ifndef"),
    StringRef("include"),
    StringRef("local"),
    StringRef("macro"),
    StringRef("patch"),
    StringRef("persistent"),
    StringRef("range"),
    StringRef("read"),
    StringRef("render"),
    StringRef("statistics"),
    StringRef("switch"),
    StringRef("undef"),
    StringRef("version"),
    StringRef("warning"),
    StringRef("while"),
    StringRef("write"),
    /* clang-format on */
};

/* POV-Ray Built-in Variables
 * list is from...
 * http://www.povray.org/documentation/view/3.7.0/212/
 */

/** Float Functions */
static StringRef text_format_pov_literals_reserved[]{
    /* clang-format on */
    StringRef("abs"),
    StringRef("acos"),
    StringRef("acosh"),
    StringRef("albedo"),
    StringRef("altitude"),
    StringRef("angle"),
    StringRef("asc"),
    StringRef("asin"),
    StringRef("asinh"),
    StringRef("atan"),
    StringRef("atan2"),
    StringRef("atand"),
    StringRef("atanh"),
    StringRef("bitwise_and"),
    StringRef("bitwise_or"),
    StringRef("bitwise_xor"),
    StringRef("blink"),
    StringRef("ceil"),
    StringRef("child"),
    StringRef("clipped_by"),
    StringRef("collect"),
    StringRef("conserve_energy"),
    StringRef("cos"),
    StringRef("cosh"),
    StringRef("crand"),
    StringRef("defined"),
    StringRef("degrees"),
    StringRef("dimension_size"),
    StringRef("dimensions"),
    StringRef("direction"),
    StringRef("div"),
    StringRef("evaluate"),
    StringRef("exp"),
    StringRef("file_exists"),
    StringRef("floor"),
    StringRef("form"),
    StringRef("function"),
    StringRef("gts_load"),
    StringRef("gts_save"),
    StringRef("inside"),
    StringRef("int"),
    StringRef("inverse"),
    StringRef("jitter"),
    StringRef("ln"),
    StringRef("load_file"),
    StringRef("location"),
    StringRef("log"),
    StringRef("look_at"),
    StringRef("matrix"),
    StringRef("max"),
    StringRef("max_intersections"),
    StringRef("max_trace"),
    StringRef("metallic"),
    StringRef("min"),
    StringRef("mod"),
    StringRef("phong_size"),
    StringRef("pow"),
    StringRef("precompute"),
    StringRef("prod"),
    StringRef("pwr"),
    StringRef("quaternion"),
    StringRef("radians"),
    StringRef("rand"),
    StringRef("reciprocal"),
    StringRef("right"),
    StringRef("rotate"),
    StringRef("roughness"),
    StringRef("save_file"),
    StringRef("scale"),
    StringRef("seed"),
    StringRef("select"),
    StringRef("shadowless"),
    StringRef("sin"),
    StringRef("sinh"),
    StringRef("sky"),
    StringRef("sqr"),
    StringRef("sqrt"),
    StringRef("strcmp"),
    StringRef("strlen"),
    StringRef("sturm"),
    StringRef("sum"),
    StringRef("tan"),
    StringRef("tanh"),
    StringRef("target"),
    StringRef("tessel"),
    StringRef("tesselate"),
    StringRef("transform"),
    StringRef("translate"),
    StringRef("turb_depth"),
    StringRef("up"),
    StringRef("val"),
    StringRef("vdot"),
    StringRef("vlength"),
    StringRef("warp"),
    StringRef("with"),

    /* Color Identifiers */
    StringRef("SRGB"),
    StringRef("blue"),
    StringRef("filter"),
    StringRef("gamma"),
    StringRef("gray"),
    StringRef("green"),
    StringRef("red"),
    StringRef("rgb"),
    StringRef("rgbf"),
    StringRef("rgbft"),
    StringRef("rgbt"),
    StringRef("sRGB"),
    StringRef("srgb"),
    StringRef("srgbf"),
    StringRef("srgbft"),
    StringRef("srgbt"),
    StringRef("transmit"),
    /* Color Spaces */
    StringRef("hsl"),
    StringRef("hsv"),
    StringRef("pov"),
    StringRef("xyl"),
    StringRef("xyv"),
    /* Vector Functions */
    StringRef("max_extent"),
    StringRef("min_extent"),
    StringRef("trace"),
    StringRef("vaxis_rotate"),
    StringRef("vcross"),
    StringRef("vnormalize"),
    StringRef("vrotate"),
    StringRef("vturbulence"),
    /* String Functions */
    StringRef("chr"),
    StringRef("concat"),
    StringRef("datetime"),
    StringRef("file_time"),
    StringRef("str"),
    StringRef("strlwr"),
    StringRef("strupr"),
    StringRef("substr"),
    StringRef("vstr"),
    /* clang-format on */
};

/* POV-Ray Built-in Variables
 * list is from...
 * http://www.povray.org/documentation/view/3.7.0/212/
 */

/* Language Keywords */
static StringRef text_format_pov_literals_builtins[]{
    /* clang-format off */
    StringRef("aa_threshold"),
    StringRef("absorption"),
    StringRef("all"),
    StringRef("all_intersections"),
    StringRef("alpha"),
    StringRef("arc_angle"),
    StringRef("area_illumination"),
    StringRef("array"),
    StringRef("bend"),
    StringRef("black_hole"),
    StringRef("color_space"),
    StringRef("colour_space"),
    StringRef("component"),
    StringRef("composite"),
    StringRef("coords"),
    StringRef("cube"),
    StringRef("cutaway_textures"),
    StringRef("dist_exp"),
    StringRef("emission"),
    StringRef("exterior"),
    StringRef("falloff_angle"),
    StringRef("file_gamma"),
    StringRef("flatness"),
    StringRef("flip"),
    StringRef("hierarchy"),
    StringRef("hypercomplex"),
    StringRef("interior"),
    StringRef("internal"),
    StringRef("iridescence"),
    StringRef("keep"),
    StringRef("lommel_seeliger"),
    StringRef("look_at"),
    StringRef("major_radius"),
    StringRef("material"),
    StringRef("max_distance"),
    StringRef("max_extent"),
    StringRef("max_iteration"),
    StringRef("metric"),
    StringRef("minnaert"),
    StringRef("move"),
    StringRef("now"),
    StringRef("offset"),
    StringRef("oren_nayar"),
    StringRef("orientation"),
    StringRef("pattern"),
    StringRef("photons"),
    StringRef("planet"),
    StringRef("pot"),
    StringRef("precision"),
    StringRef("radiosity"),
    StringRef("reflection"),
    StringRef("reflection_exponent"),
    StringRef("refraction"),
    StringRef("repeat"),
    StringRef("roll"),
    StringRef("scattering"),
    StringRef("screw"),
    StringRef("size"),
    StringRef("slice"),
    StringRef("smooth"),
    StringRef("smooth_triangle"),
    StringRef("solid"),
    StringRef("subsurface"),
    StringRef("texture"),
    StringRef("tolerance"),
    StringRef("toroidal"),
    StringRef("type"),
    StringRef("water_level"),
    StringRef("width"),

    /* Animation Options */
    StringRef("clock"),
    StringRef("clock_delta"),
    StringRef("clock_on"),
    StringRef("final_clock"),
    StringRef("final_frame"),
    StringRef("frame_number"),
    StringRef("global_settings"),
    StringRef("image_height"),
    StringRef("image_width"),
    StringRef("initial_clock"),
    StringRef("initial_frame"),
    StringRef("input_file_name"),

    /* Spline Identifiers */
    StringRef("akima_spline"),
    StringRef("b_spline"),
    StringRef("basic_x_spline"),
    StringRef("bezier_spline"),
    StringRef("conic_sweep"),
    StringRef("cubic_spline"),
    StringRef("extended_x_spline"),
    StringRef("general_x_spline"),
    StringRef("linear_spline"),
    StringRef("linear_sweep"),
    StringRef("natural_spline"),
    StringRef("quadratic_spline"),
    StringRef("sor_spline"),
    StringRef("tcb_spline"),

    /* Patterns */
    StringRef("agate"),
    StringRef("aoi"),
    StringRef("average"),
    StringRef("binary"),
    StringRef("boxed"),
    StringRef("bozo"),
    StringRef("brick"),
    StringRef("bump_map"),
    StringRef("bumps"),
    StringRef("cells"),
    StringRef("checker"),
    StringRef("crackle"),
    StringRef("cubic"),
    StringRef("cylindrical"),
    StringRef("density_file"),
    StringRef("dents"),
    StringRef("facets"),
    StringRef("gradient"),
    StringRef("granite"),
    StringRef("hexagon"),
    StringRef("image_map"),
    StringRef("image_pattern"),
    StringRef("julia"),
    StringRef("leopard"),
    StringRef("magnet"),
    StringRef("mandel"),
    StringRef("marble"),
    StringRef("masonry"),
    StringRef("onion"),
    StringRef("pavement"),
    StringRef("pigment_pattern"),
    StringRef("planar"),
    StringRef("proportion"),
    StringRef("proximity"),
    StringRef("quilted"),
    StringRef("radial"),
    StringRef("ripples"),
    StringRef("slope"),
    StringRef("spherical"),
    StringRef("spiral1"),
    StringRef("spiral2"),
    StringRef("spotted"),
    StringRef("square"),
    StringRef("tile2"),
    StringRef("tiles"),
    StringRef("tiling"),
    StringRef("triangular"),
    StringRef("voronoi"),
    StringRef("waves"),
    StringRef("wood"),
    StringRef("wrinkles"),

    /* Objects */
    StringRef("ambient"),
    StringRef("background"),
    StringRef("bicubic_patch"),
    StringRef("blob"),
    StringRef("box"),
    StringRef("brilliance"),
    StringRef("camera"),
    StringRef("color"),
    StringRef("colour"),
    StringRef("cone"),
    StringRef("cubic"),
    StringRef("cubic_spline"),
    StringRef("cylinder"),
    StringRef("difference"),
    StringRef("diffuse"),
    StringRef("disc"),
    StringRef("displace"),
    StringRef("fog"),
    StringRef("galley"),
    StringRef("height_field"),
    StringRef("intermerge"),
    StringRef("intersection"),
    StringRef("interunion"),
    StringRef("irid"),
    StringRef("isosurface"),
    StringRef("julia_fractal"),
    StringRef("lathe"),
    StringRef("lemon"),
    StringRef("light_group"),
    StringRef("light_source"),
    StringRef("media"),
    StringRef("merge"),
    StringRef("mesh"),
    StringRef("mesh2"),
    StringRef("object"),
    StringRef("ovus"),
    StringRef("parametric"),
    StringRef("phong"),
    StringRef("plane"),
    StringRef("poly"),
    StringRef("polygon"),
    StringRef("polynomial"),
    StringRef("prism"),
    StringRef("quadric"),
    StringRef("quartic"),
    StringRef("rainbow"),
    StringRef("sky_sphere"),
    StringRef("sor"),
    StringRef("specular"),
    StringRef("sphere"),
    StringRef("sphere_sweep"),
    StringRef("spline"),
    StringRef("superellipsoid"),
    StringRef("text"),
    StringRef("torus"),
    StringRef("triangle"),
    StringRef("union"),
    /* Built-in Vectors */
    StringRef("t"),
    StringRef("u"),
    StringRef("v"),
    StringRef("x"),
    StringRef("y"),
    StringRef("z"),
    /* clang-format on */
};

/**
 * POV modifiers.
 * See:
 * http://www.povray.org/documentation/view/3.7.0/212/
 */
static StringRef text_format_pov_literals_specialvar[]{
    /* clang-format off */
    StringRef("aa_level"),
    StringRef("accuracy"),
    StringRef("accuracy"),
    StringRef("adaptive"),
    StringRef("adc_bailout"),
    StringRef("agate_turb"),
    StringRef("albinos"),
    StringRef("always_sample"),
    StringRef("ambient_light"),
    StringRef("amount"),
    StringRef("assumed_gamma"),
    StringRef("autostop"),
    StringRef("bounded_by"),
    StringRef("brick_size"),
    StringRef("brightness"),
    StringRef("bump_size"),
    StringRef("caustics"),
    StringRef("charset"),
    StringRef("circular"),
    StringRef("color_map"),
    StringRef("colour_map"),
    StringRef("contained_by"),
    StringRef("control0"),
    StringRef("control1"),
    StringRef("count"),
    StringRef("cubic"),
    StringRef("cubic_wave"),
    StringRef("density"),
    StringRef("density_map"),
    StringRef("dispersion"),
    StringRef("dispersion_samples"),
    StringRef("distance"),
    StringRef("double_illuminate"),
    StringRef("eccentricity"),
    StringRef("error_bound"),
    StringRef("expand_thresholds"),
    StringRef("exponent"),
    StringRef("extinction"),
    StringRef("face_indices"),
    StringRef("fade_color"),
    StringRef("fade_colour"),
    StringRef("fade_distance"),
    StringRef("fade_power"),
    StringRef("finish"),
    StringRef("fixed"),
    StringRef("fog_alt"),
    StringRef("fog_offset"),
    StringRef("fog_type"),
    StringRef("frequency"),
    StringRef("fresnel"),
    StringRef("gather"),
    StringRef("global_lights"),
    StringRef("gray_threshold"),
    StringRef("hf_gray_16"),
    StringRef("hollow"),
    StringRef("importance"),
    StringRef("inbound"),
    StringRef("inner"),
    StringRef("inside_point"),
    StringRef("inside_vector"),
    StringRef("interior_texture"),
    StringRef("interpolate"),
    StringRef("intervals"),
    StringRef("ior"),
    StringRef("irid_wavelength"),
    StringRef("lambda"),
    StringRef("low_error_factor"),
    StringRef("map_type"),
    StringRef("material_map"),
    StringRef("max_gradient"),
    StringRef("max_sample"),
    StringRef("max_trace_level"),
    StringRef("maximal"),
    StringRef("maximum_reuse"),
    StringRef("media_attenuation"),
    StringRef("media_interaction"),
    StringRef("method"),
    StringRef("minimal"),
    StringRef("minimum_reuse"),
    StringRef("mm_per_unit"),
    StringRef("modulation"),
    StringRef("mortar"),
    StringRef("nearest_count"),
    StringRef("no_bump_scale"),
    StringRef("no_cache"),
    StringRef("no_image"),
    StringRef("no_radiosity"),
    StringRef("no_reflection"),
    StringRef("no_shadow"),
    StringRef("noise_generator"),
    StringRef("normal"),
    StringRef("normal_indices"),
    StringRef("normal_map"),
    StringRef("normal_vectors"),
    StringRef("number_of_waves"),
    StringRef("octaves"),
    StringRef("offset"),
    StringRef("omega"),
    StringRef("once"),
    StringRef("open"),
    StringRef("orient"),
    StringRef("origin"),
    StringRef("original"),
    StringRef("outbound"),
    StringRef("outside"),
    StringRef("pass_through"),
    StringRef("phase"),
    StringRef("pigment"),
    StringRef("pigment_map"),
    StringRef("polarity"),
    StringRef("poly_wave"),
    StringRef("precision"),
    StringRef("pretrace_end"),
    StringRef("pretrace_start"),
    StringRef("projected_through"),
    StringRef("quick_color"),
    StringRef("quick_colour"),
    StringRef("ramp_wave"),
    StringRef("ratio"),
    StringRef("recursion_limit"),
    StringRef("samples"),
    StringRef("scallop_wave"),
    StringRef("sine_wave"),
    StringRef("slope_map"),
    StringRef("spacing"),
    StringRef("split_union"),
    StringRef("strength"),
    StringRef("texture_list"),
    StringRef("texture_map"),
    StringRef("thickness"),
    StringRef("threshold"),
    StringRef("translucency"),
    StringRef("triangle_wave"),
    StringRef("turbulence"),
    StringRef("u_steps"),
    StringRef("use_alpha"),
    StringRef("use_color"),
    StringRef("use_colour"),
    StringRef("use_index"),
    StringRef("uv_indices"),
    StringRef("uv_mapping"),
    StringRef("uv_vectors"),
    StringRef("v_steps"),
    StringRef("vertex_vectors"),

    /* Light Types and options. */
    StringRef("area_light"),
    StringRef("fade_power"),
    StringRef("falloff"),
    StringRef("looks_like"),
    StringRef("parallel"),
    StringRef("point_at"),
    StringRef("radius"),
    StringRef("spotlight"),
    StringRef("tightness"),

    /* Camera Types and options. */
    StringRef("aitoff_hammer"),
    StringRef("aperture"),
    StringRef("balthasart"),
    StringRef("behrmann"),
    StringRef("blur_samples"),
    StringRef("camera_direction"),
    StringRef("camera_location"),
    StringRef("camera_right"),
    StringRef("camera_type"),
    StringRef("camera_up"),
    StringRef("confidence"),
    StringRef("eckert_iv"),
    StringRef("eckert_vi"),
    StringRef("edwards"),
    StringRef("fisheye"),
    StringRef("focal_point"),
    StringRef("gall"),
    StringRef("hobo_dyer"),
    StringRef("icosa"),
    StringRef("lambert_azimuthal"),
    StringRef("lambert_cylindrical"),
    StringRef("mercator"),
    StringRef("mesh_camera"),
    StringRef("miller_cylindrical"),
    StringRef("mollweide"),
    StringRef("octa"),
    StringRef("omni_directional_stereo"),
    StringRef("omnimax"),
    StringRef("orthographic"),
    StringRef("panoramic"),
    StringRef("parallaxe"),
    StringRef("perspective"),
    StringRef("peters"),
    StringRef("plate_carree"),
    StringRef("smyth_craster"),
    StringRef("stereo"),
    StringRef("tetra"),
    StringRef("ultra_wide_angle"),
    StringRef("van_der_grinten"),
    StringRef("variance"),
    /* clang-format on */
};

/** POV Built-in Constants. */
static StringRef text_format_pov_literals_bool[]{
    /* clang-format off */
    StringRef("false"),
    StringRef("no"),
    StringRef("off"),
    StringRef("on"),
    StringRef("pi"),
    StringRef("tau"),
    StringRef("true"),
    StringRef("unofficial"),
    StringRef("yes"),
    /* Encodings. */
    StringRef("ascii"),
    StringRef("bt2020"),
    StringRef("bt709"),
    StringRef("sint16be"),
    StringRef("sint16le"),
    StringRef("sint32be"),
    StringRef("sint32le"),
    StringRef("sint8"),
    StringRef("uint16be"),
    StringRef("uint16le"),
    StringRef("uint8"),
    StringRef("utf8"),
    /* File-types. */
    StringRef("df3"),
    StringRef("exr"),
    StringRef("gif"),
    StringRef("hdr"),
    StringRef("iff"),
    StringRef("jpeg"),
    StringRef("pgm"),
    StringRef("png"),
    StringRef("ppm"),
    StringRef("sys"),
    StringRef("tga"),
    StringRef("tiff"),
    StringRef("ttf"),
    /* clang-format on */
};

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
      text_format_pov_literals_keyword, string);
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
      text_format_pov_literals_reserved, string);
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
      text_format_pov_literals_builtins, string);
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
      text_format_pov_literals_specialvar, string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_pov_find_bool(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(text_format_pov_literals_bool,
                                                                    string);
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

  text_format_string_literals_sort_for_lookup(text_format_pov_literals_keyword);
  text_format_string_literals_sort_for_lookup(text_format_pov_literals_reserved);
  text_format_string_literals_sort_for_lookup(text_format_pov_literals_builtins);
  text_format_string_literals_sort_for_lookup(text_format_pov_literals_specialvar);
  text_format_string_literals_sort_for_lookup(text_format_pov_literals_bool);
}

/** \} */
