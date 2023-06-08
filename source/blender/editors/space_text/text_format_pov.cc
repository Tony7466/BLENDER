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
 * Checks the specified source string for a POV keyword (minus boolean & 'nil').
 * This name must start at the beginning of the source string and must be
 * followed by a non-identifier (see #text_check_identifier(char)) or null char.
 *
 * If a keyword is found, the length of the matching word is returned.
 * Otherwise, -1 is returned.
 *
 * See:
 * http://www.povray.org/documentation/view/3.7.0/212/
 */
static int txtfmt_pov_find_keyword(const char *string)
{
  /* Keep aligned args for readability. */

  int i;
  /* Language Directives */
  constexpr keyword_info keywords[]{
      KEYWORD_INFO("deprecated"), KEYWORD_INFO("persistent"), KEYWORD_INFO("statistics"),
      KEYWORD_INFO("version"),    KEYWORD_INFO("warning"),    KEYWORD_INFO("declare"),
      KEYWORD_INFO("default"),    KEYWORD_INFO("include"),    KEYWORD_INFO("append"),
      KEYWORD_INFO("elseif"),     KEYWORD_INFO("debug"),      KEYWORD_INFO("break"),
      KEYWORD_INFO("else"),       KEYWORD_INFO("error"),      KEYWORD_INFO("fclose"),
      KEYWORD_INFO("fopen"),      KEYWORD_INFO("ifndef"),     KEYWORD_INFO("ifdef"),
      KEYWORD_INFO("patch"),      KEYWORD_INFO("local"),      KEYWORD_INFO("macro"),
      KEYWORD_INFO("range"),      KEYWORD_INFO("read"),       KEYWORD_INFO("render"),
      KEYWORD_INFO("switch"),     KEYWORD_INFO("undef"),      KEYWORD_INFO("while"),
      KEYWORD_INFO("write"),      KEYWORD_INFO("case"),       KEYWORD_INFO("end"),
      KEYWORD_INFO("for"),        KEYWORD_INFO("if")};

  i = find_keyword_length(keywords, string);

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_pov_find_reserved_keywords(const char *string)
{
  int i;
  /* POV-Ray Built-in Variables
   * list is from...
   * http://www.povray.org/documentation/view/3.7.0/212/
   */

  /* Keep aligned args for readability. */

  /* Float Functions */
  constexpr keyword_info keywords[]{
      KEYWORD_INFO("conserve_energy"),
      KEYWORD_INFO("max_intersections"),
      KEYWORD_INFO("dimension_size"),
      KEYWORD_INFO("bitwise_and"),
      KEYWORD_INFO("bitwise_or"),
      KEYWORD_INFO("bitwise_xor"),
      KEYWORD_INFO("file_exists"),
      KEYWORD_INFO("precompute"),
      KEYWORD_INFO("dimensions"),
      KEYWORD_INFO("clipped_by"),
      KEYWORD_INFO("shadowless"),
      KEYWORD_INFO("turb_depth"),
      KEYWORD_INFO("reciprocal"),
      KEYWORD_INFO("quaternion"),
      KEYWORD_INFO("phong_size"),
      KEYWORD_INFO("tesselate"),
      KEYWORD_INFO("save_file"),
      KEYWORD_INFO("load_file"),
      KEYWORD_INFO("max_trace"),
      KEYWORD_INFO("transform"),
      KEYWORD_INFO("translate"),
      KEYWORD_INFO("direction"),
      KEYWORD_INFO("roughness"),
      KEYWORD_INFO("metallic"),
      KEYWORD_INFO("gts_load"),
      KEYWORD_INFO("gts_save"),
      KEYWORD_INFO("location"),
      KEYWORD_INFO("altitude"),
      KEYWORD_INFO("function"),
      KEYWORD_INFO("evaluate"),
      KEYWORD_INFO("inverse"),
      KEYWORD_INFO("collect"),
      KEYWORD_INFO("target"),
      KEYWORD_INFO("albedo"),
      KEYWORD_INFO("rotate"),
      KEYWORD_INFO("matrix"),
      KEYWORD_INFO("look_at"),
      KEYWORD_INFO("jitter"),
      KEYWORD_INFO("angle"),
      KEYWORD_INFO("right"),
      KEYWORD_INFO("scale"),
      KEYWORD_INFO("child"),
      KEYWORD_INFO("crand"),
      KEYWORD_INFO("blink"),
      KEYWORD_INFO("defined"),
      KEYWORD_INFO("degrees"),
      KEYWORD_INFO("inside"),
      KEYWORD_INFO("radians"),
      KEYWORD_INFO("vlength"),
      KEYWORD_INFO("select"),
      KEYWORD_INFO("floor"),
      KEYWORD_INFO("strcmp"),
      KEYWORD_INFO("strlen"),
      KEYWORD_INFO("tessel"),
      KEYWORD_INFO("sturm"),
      KEYWORD_INFO("abs"),
      KEYWORD_INFO("acosh"),
      KEYWORD_INFO("prod"),
      KEYWORD_INFO("with"),
      KEYWORD_INFO("acos"),
      KEYWORD_INFO("asc"),
      KEYWORD_INFO("asinh"),
      KEYWORD_INFO("asin"),
      KEYWORD_INFO("atan2"),
      KEYWORD_INFO("atand"),
      KEYWORD_INFO("atanh"),
      KEYWORD_INFO("atan"),
      KEYWORD_INFO("ceil"),
      KEYWORD_INFO("warp"),
      KEYWORD_INFO("cosh"),
      KEYWORD_INFO("log"),
      KEYWORD_INFO("max"),
      KEYWORD_INFO("min"),
      KEYWORD_INFO("mod"),
      KEYWORD_INFO("pow"),
      KEYWORD_INFO("rand"),
      KEYWORD_INFO("seed"),
      KEYWORD_INFO("form"),
      KEYWORD_INFO("sinh"),
      KEYWORD_INFO("sqrt"),
      KEYWORD_INFO("tanh"),
      KEYWORD_INFO("vdot"),
      KEYWORD_INFO("sin"),
      KEYWORD_INFO("sqr"),
      KEYWORD_INFO("sum"),
      KEYWORD_INFO("pwr"),
      KEYWORD_INFO("tan"),
      KEYWORD_INFO("val"),
      KEYWORD_INFO("cos"),
      KEYWORD_INFO("div"),
      KEYWORD_INFO("exp"),
      KEYWORD_INFO("int"),
      KEYWORD_INFO("sky"),
      KEYWORD_INFO("up"),
      KEYWORD_INFO("ln"),

      /* Color Identifiers */
      KEYWORD_INFO("transmit"),
      KEYWORD_INFO("filter"),
      KEYWORD_INFO("srgbft"),
      KEYWORD_INFO("srgbf"),
      KEYWORD_INFO("srgbt"),
      KEYWORD_INFO("rgbft"),
      KEYWORD_INFO("gamma"),
      KEYWORD_INFO("green"),
      KEYWORD_INFO("blue"),
      KEYWORD_INFO("gray"),
      KEYWORD_INFO("srgb"),
      KEYWORD_INFO("sRGB"),
      KEYWORD_INFO("SRGB"),
      KEYWORD_INFO("rgbf"),
      KEYWORD_INFO("rgbt"),
      KEYWORD_INFO("rgb"),
      KEYWORD_INFO("red"),
      /* Color Spaces */
      KEYWORD_INFO("pov"),
      KEYWORD_INFO("hsl"),
      KEYWORD_INFO("hsv"),
      KEYWORD_INFO("xyl"),
      KEYWORD_INFO("xyv"),
      /* Vector Functions */
      KEYWORD_INFO("vaxis_rotate"),
      KEYWORD_INFO("vturbulence"),
      KEYWORD_INFO("min_extent"),
      KEYWORD_INFO("vnormalize"),
      KEYWORD_INFO("max_extent"),
      KEYWORD_INFO("vrotate"),
      KEYWORD_INFO("vcross"),
      KEYWORD_INFO("trace"),
      /* String Functions */
      KEYWORD_INFO("file_time"),
      KEYWORD_INFO("datetime"),
      KEYWORD_INFO("concat"),
      KEYWORD_INFO("strlwr"),
      KEYWORD_INFO("strupr"),
      KEYWORD_INFO("substr"),
      KEYWORD_INFO("vstr"),
      KEYWORD_INFO("chr"),
      KEYWORD_INFO("str"),
  };

  i = find_keyword_length(keywords, string);

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_pov_find_reserved_builtins(const char *string)
{
  int i;

  /* POV-Ray Built-in Variables
   * list is from...
   * http://www.povray.org/documentation/view/3.7.0/212/
   */

  /* Keep aligned args for readability. */

  /* Language Keywords */
  constexpr keyword_info keywords[]{KEYWORD_INFO("reflection_exponent"),
                                KEYWORD_INFO("area_illumination"),
                                KEYWORD_INFO("all_intersections"),
                                KEYWORD_INFO("cutaway_textures"),
                                KEYWORD_INFO("smooth_triangle"),
                                KEYWORD_INFO("lommel_seeliger"),
                                KEYWORD_INFO("falloff_angle"),
                                KEYWORD_INFO("aa_threshold"),
                                KEYWORD_INFO("hypercomplex"),
                                KEYWORD_INFO("major_radius"),
                                KEYWORD_INFO("max_distance"),
                                KEYWORD_INFO("max_iteration"),
                                KEYWORD_INFO("colour_space"),
                                KEYWORD_INFO("color_space"),
                                KEYWORD_INFO("iridescence"),
                                KEYWORD_INFO("subsurface"),
                                KEYWORD_INFO("scattering"),
                                KEYWORD_INFO("absorption"),
                                KEYWORD_INFO("water_level"),
                                KEYWORD_INFO("reflection"),
                                KEYWORD_INFO("max_extent"),
                                KEYWORD_INFO("oren_nayar"),
                                KEYWORD_INFO("refraction"),
                                KEYWORD_INFO("hierarchy"),
                                KEYWORD_INFO("radiosity"),
                                KEYWORD_INFO("tolerance"),
                                KEYWORD_INFO("interior"),
                                KEYWORD_INFO("toroidal"),
                                KEYWORD_INFO("emission"),
                                KEYWORD_INFO("material"),
                                KEYWORD_INFO("internal"),
                                KEYWORD_INFO("photons"),
                                KEYWORD_INFO("arc_angle"),
                                KEYWORD_INFO("minnaert"),
                                KEYWORD_INFO("texture"),
                                KEYWORD_INFO("array"),
                                KEYWORD_INFO("black_hole"),
                                KEYWORD_INFO("component"),
                                KEYWORD_INFO("composite"),
                                KEYWORD_INFO("coords"),
                                KEYWORD_INFO("cube"),
                                KEYWORD_INFO("dist_exp"),
                                KEYWORD_INFO("exterior"),
                                KEYWORD_INFO("file_gamma"),
                                KEYWORD_INFO("flatness"),
                                KEYWORD_INFO("planet"),
                                KEYWORD_INFO("screw"),
                                KEYWORD_INFO("keep"),
                                KEYWORD_INFO("flip"),
                                KEYWORD_INFO("move"),
                                KEYWORD_INFO("roll"),
                                KEYWORD_INFO("look_at"),
                                KEYWORD_INFO("metric"),
                                KEYWORD_INFO("offset"),
                                KEYWORD_INFO("orientation"),
                                KEYWORD_INFO("pattern"),
                                KEYWORD_INFO("precision"),
                                KEYWORD_INFO("width"),
                                KEYWORD_INFO("repeat"),
                                KEYWORD_INFO("bend"),
                                KEYWORD_INFO("size"),
                                KEYWORD_INFO("alpha"),
                                KEYWORD_INFO("slice"),
                                KEYWORD_INFO("smooth"),
                                KEYWORD_INFO("solid"),
                                KEYWORD_INFO("all"),
                                KEYWORD_INFO("now"),
                                KEYWORD_INFO("pot"),
                                KEYWORD_INFO("type"),

                                /* Animation Options */
                                KEYWORD_INFO("global_settings"),
                                KEYWORD_INFO("input_file_name"),
                                KEYWORD_INFO("initial_clock"),
                                KEYWORD_INFO("initial_frame"),
                                KEYWORD_INFO("frame_number"),
                                KEYWORD_INFO("image_height"),
                                KEYWORD_INFO("image_width"),
                                KEYWORD_INFO("final_clock"),
                                KEYWORD_INFO("final_frame"),
                                KEYWORD_INFO("clock_delta"),
                                KEYWORD_INFO("clock_on"),
                                KEYWORD_INFO("clock"),

                                /* Spline Identifiers */
                                KEYWORD_INFO("extended_x_spline"),
                                KEYWORD_INFO("general_x_spline"),
                                KEYWORD_INFO("quadratic_spline"),
                                KEYWORD_INFO("basic_x_spline"),
                                KEYWORD_INFO("natural_spline"),
                                KEYWORD_INFO("linear_spline"),
                                KEYWORD_INFO("bezier_spline"),
                                KEYWORD_INFO("akima_spline"),
                                KEYWORD_INFO("cubic_spline"),
                                KEYWORD_INFO("sor_spline"),
                                KEYWORD_INFO("tcb_spline"),
                                KEYWORD_INFO("linear_sweep"),
                                KEYWORD_INFO("conic_sweep"),
                                KEYWORD_INFO("b_spline"),

                                /* Patterns */
                                KEYWORD_INFO("pigment_pattern"),
                                KEYWORD_INFO("image_pattern"),
                                KEYWORD_INFO("density_file"),
                                KEYWORD_INFO("cylindrical"),
                                KEYWORD_INFO("proportion"),
                                KEYWORD_INFO("triangular"),
                                KEYWORD_INFO("image_map"),
                                KEYWORD_INFO("proximity"),
                                KEYWORD_INFO("spherical"),
                                KEYWORD_INFO("bump_map"),
                                KEYWORD_INFO("wrinkles"),
                                KEYWORD_INFO("average"),
                                KEYWORD_INFO("voronoi"),
                                KEYWORD_INFO("masonry"),
                                KEYWORD_INFO("binary"),
                                KEYWORD_INFO("boxed"),
                                KEYWORD_INFO("bozo"),
                                KEYWORD_INFO("brick"),
                                KEYWORD_INFO("bumps"),
                                KEYWORD_INFO("cells"),
                                KEYWORD_INFO("checker"),
                                KEYWORD_INFO("crackle"),
                                KEYWORD_INFO("cubic"),
                                KEYWORD_INFO("dents"),
                                KEYWORD_INFO("facets"),
                                KEYWORD_INFO("gradient"),
                                KEYWORD_INFO("granite"),
                                KEYWORD_INFO("hexagon"),
                                KEYWORD_INFO("julia"),
                                KEYWORD_INFO("leopard"),
                                KEYWORD_INFO("magnet"),
                                KEYWORD_INFO("mandel"),
                                KEYWORD_INFO("marble"),
                                KEYWORD_INFO("onion"),
                                KEYWORD_INFO("pavement"),
                                KEYWORD_INFO("planar"),
                                KEYWORD_INFO("quilted"),
                                KEYWORD_INFO("radial"),
                                KEYWORD_INFO("ripples"),
                                KEYWORD_INFO("slope"),
                                KEYWORD_INFO("spiral1"),
                                KEYWORD_INFO("spiral2"),
                                KEYWORD_INFO("spotted"),
                                KEYWORD_INFO("square"),
                                KEYWORD_INFO("tile2"),
                                KEYWORD_INFO("tiling"),
                                KEYWORD_INFO("tiles"),
                                KEYWORD_INFO("waves"),
                                KEYWORD_INFO("wood"),
                                KEYWORD_INFO("agate"),
                                KEYWORD_INFO("aoi"),

                                /* Objects */
                                KEYWORD_INFO("superellipsoid"),
                                KEYWORD_INFO("bicubic_patch"),
                                KEYWORD_INFO("julia_fractal"),
                                KEYWORD_INFO("height_field"),
                                KEYWORD_INFO("cubic_spline"),
                                KEYWORD_INFO("sphere_sweep"),
                                KEYWORD_INFO("light_group"),
                                KEYWORD_INFO("light_source"),
                                KEYWORD_INFO("intersection"),
                                KEYWORD_INFO("isosurface"),
                                KEYWORD_INFO("background"),
                                KEYWORD_INFO("sky_sphere"),
                                KEYWORD_INFO("cylinder"),
                                KEYWORD_INFO("difference"),
                                KEYWORD_INFO("brilliance"),
                                KEYWORD_INFO("parametric"),
                                KEYWORD_INFO("interunion"),
                                KEYWORD_INFO("intermerge"),
                                KEYWORD_INFO("polynomial"),
                                KEYWORD_INFO("displace"),
                                KEYWORD_INFO("specular"),
                                KEYWORD_INFO("ambient"),
                                KEYWORD_INFO("diffuse"),
                                KEYWORD_INFO("polygon"),
                                KEYWORD_INFO("quadric"),
                                KEYWORD_INFO("quartic"),
                                KEYWORD_INFO("rainbow"),
                                KEYWORD_INFO("sphere"),
                                KEYWORD_INFO("spline"),
                                KEYWORD_INFO("prism"),
                                KEYWORD_INFO("camera"),
                                KEYWORD_INFO("galley"),
                                KEYWORD_INFO("cubic"),
                                KEYWORD_INFO("phong"),
                                KEYWORD_INFO("cone"),
                                KEYWORD_INFO("blob"),
                                KEYWORD_INFO("box"),
                                KEYWORD_INFO("disc"),
                                KEYWORD_INFO("fog"),
                                KEYWORD_INFO("lathe"),
                                KEYWORD_INFO("merge"),
                                KEYWORD_INFO("mesh2"),
                                KEYWORD_INFO("mesh"),
                                KEYWORD_INFO("object"),
                                KEYWORD_INFO("ovus"),
                                KEYWORD_INFO("lemon"),
                                KEYWORD_INFO("plane"),
                                KEYWORD_INFO("poly"),
                                KEYWORD_INFO("irid"),
                                KEYWORD_INFO("sor"),
                                KEYWORD_INFO("text"),
                                KEYWORD_INFO("torus"),
                                KEYWORD_INFO("triangle"),
                                KEYWORD_INFO("union"),
                                KEYWORD_INFO("colour"),
                                KEYWORD_INFO("color"),
                                KEYWORD_INFO("media"),
                                /* Built-in Vectors */
                                KEYWORD_INFO("t"),
                                KEYWORD_INFO("u"),
                                KEYWORD_INFO("v"),
                                KEYWORD_INFO("x"),
                                KEYWORD_INFO("y"),
                                KEYWORD_INFO("z")};

  i = find_keyword_length(keywords, string);

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

/**
 * Checks the specified source string for a POV modifiers. This
 * name must start at the beginning of the source string and must be followed
 * by a non-identifier (see #text_check_identifier(char)) or null character.
 *
 * If a special name is found, the length of the matching name is returned.
 * Otherwise, -1 is returned.
 *
 * See:
 * http://www.povray.org/documentation/view/3.7.0/212/
 */
static int txtfmt_pov_find_specialvar(const char *string)
{
  int i;

  /* Keep aligned args for readability. */

  /* Modifiers */
  constexpr keyword_info keywords[]{KEYWORD_INFO("dispersion_samples"),
                                KEYWORD_INFO("projected_through"),
                                KEYWORD_INFO("double_illuminate"),
                                KEYWORD_INFO("expand_thresholds"),
                                KEYWORD_INFO("media_interaction"),
                                KEYWORD_INFO("media_attenuation"),
                                KEYWORD_INFO("low_error_factor"),
                                KEYWORD_INFO("recursion_limit"),
                                KEYWORD_INFO("interior_texture"),
                                KEYWORD_INFO("max_trace_level"),
                                KEYWORD_INFO("gray_threshold"),
                                KEYWORD_INFO("pretrace_start"),
                                KEYWORD_INFO("normal_indices"),
                                KEYWORD_INFO("normal_vectors"),
                                KEYWORD_INFO("vertex_vectors"),
                                KEYWORD_INFO("noise_generator"),
                                KEYWORD_INFO("irid_wavelength"),
                                KEYWORD_INFO("number_of_waves"),
                                KEYWORD_INFO("ambient_light"),
                                KEYWORD_INFO("inside_vector"),
                                KEYWORD_INFO("face_indices"),
                                KEYWORD_INFO("texture_list"),
                                KEYWORD_INFO("max_gradient"),
                                KEYWORD_INFO("uv_indices"),
                                KEYWORD_INFO("uv_vectors"),
                                KEYWORD_INFO("fade_distance"),
                                KEYWORD_INFO("global_lights"),
                                KEYWORD_INFO("no_bump_scale"),
                                KEYWORD_INFO("pretrace_end"),
                                KEYWORD_INFO("no_radiosity"),
                                KEYWORD_INFO("no_reflection"),
                                KEYWORD_INFO("assumed_gamma"),
                                KEYWORD_INFO("scallop_wave"),
                                KEYWORD_INFO("triangle_wave"),
                                KEYWORD_INFO("nearest_count"),
                                KEYWORD_INFO("maximum_reuse"),
                                KEYWORD_INFO("minimum_reuse"),
                                KEYWORD_INFO("always_sample"),
                                KEYWORD_INFO("translucency"),
                                KEYWORD_INFO("eccentricity"),
                                KEYWORD_INFO("contained_by"),
                                KEYWORD_INFO("inside_point"),
                                KEYWORD_INFO("adc_bailout"),
                                KEYWORD_INFO("density_map"),
                                KEYWORD_INFO("split_union"),
                                KEYWORD_INFO("mm_per_unit"),
                                KEYWORD_INFO("agate_turb"),
                                KEYWORD_INFO("bounded_by"),
                                KEYWORD_INFO("brick_size"),
                                KEYWORD_INFO("hf_gray_16"),
                                KEYWORD_INFO("dispersion"),
                                KEYWORD_INFO("extinction"),
                                KEYWORD_INFO("thickness"),
                                KEYWORD_INFO("color_map"),
                                KEYWORD_INFO("colour_map"),
                                KEYWORD_INFO("cubic_wave"),
                                KEYWORD_INFO("fade_colour"),
                                KEYWORD_INFO("fade_power"),
                                KEYWORD_INFO("fade_color"),
                                KEYWORD_INFO("normal_map"),
                                KEYWORD_INFO("pigment_map"),
                                KEYWORD_INFO("quick_color"),
                                KEYWORD_INFO("quick_colour"),
                                KEYWORD_INFO("material_map"),
                                KEYWORD_INFO("pass_through"),
                                KEYWORD_INFO("interpolate"),
                                KEYWORD_INFO("texture_map"),
                                KEYWORD_INFO("error_bound"),
                                KEYWORD_INFO("brightness"),
                                KEYWORD_INFO("use_color"),
                                KEYWORD_INFO("use_alpha"),
                                KEYWORD_INFO("use_colour"),
                                KEYWORD_INFO("use_index"),
                                KEYWORD_INFO("uv_mapping"),
                                KEYWORD_INFO("importance"),
                                KEYWORD_INFO("max_sample"),
                                KEYWORD_INFO("intervals"),
                                KEYWORD_INFO("sine_wave"),
                                KEYWORD_INFO("slope_map"),
                                KEYWORD_INFO("poly_wave"),
                                KEYWORD_INFO("no_shadow"),
                                KEYWORD_INFO("ramp_wave"),
                                KEYWORD_INFO("precision"),
                                KEYWORD_INFO("original"),
                                KEYWORD_INFO("accuracy"),
                                KEYWORD_INFO("map_type"),
                                KEYWORD_INFO("no_image"),
                                KEYWORD_INFO("distance"),
                                KEYWORD_INFO("autostop"),
                                KEYWORD_INFO("caustics"),

                                KEYWORD_INFO("octaves"),
                                KEYWORD_INFO("aa_level"),
                                KEYWORD_INFO("frequency"),
                                KEYWORD_INFO("fog_offset"),
                                KEYWORD_INFO("modulation"),
                                KEYWORD_INFO("outbound"),
                                KEYWORD_INFO("no_cache"),
                                KEYWORD_INFO("pigment"),
                                KEYWORD_INFO("charset"),
                                KEYWORD_INFO("inbound"),
                                KEYWORD_INFO("outside"),
                                KEYWORD_INFO("inner"),
                                KEYWORD_INFO("turbulence"),
                                KEYWORD_INFO("threshold"),
                                KEYWORD_INFO("accuracy"),
                                KEYWORD_INFO("polarity"),
                                KEYWORD_INFO("bump_size"),
                                KEYWORD_INFO("circular"),
                                KEYWORD_INFO("control0"),
                                KEYWORD_INFO("control1"),
                                KEYWORD_INFO("maximal"),
                                KEYWORD_INFO("minimal"),
                                KEYWORD_INFO("fog_type"),
                                KEYWORD_INFO("fog_alt"),
                                KEYWORD_INFO("samples"),
                                KEYWORD_INFO("origin"),
                                KEYWORD_INFO("amount"),
                                KEYWORD_INFO("adaptive"),
                                KEYWORD_INFO("exponent"),
                                KEYWORD_INFO("strength"),
                                KEYWORD_INFO("density"),
                                KEYWORD_INFO("fresnel"),
                                KEYWORD_INFO("albinos"),
                                KEYWORD_INFO("finish"),
                                KEYWORD_INFO("method"),
                                KEYWORD_INFO("omega"),
                                KEYWORD_INFO("fixed"),
                                KEYWORD_INFO("spacing"),
                                KEYWORD_INFO("u_steps"),
                                KEYWORD_INFO("v_steps"),
                                KEYWORD_INFO("offset"),
                                KEYWORD_INFO("hollow"),
                                KEYWORD_INFO("gather"),
                                KEYWORD_INFO("lambda"),
                                KEYWORD_INFO("mortar"),
                                KEYWORD_INFO("cubic"),
                                KEYWORD_INFO("count"),
                                KEYWORD_INFO("once"),
                                KEYWORD_INFO("orient"),
                                KEYWORD_INFO("normal"),
                                KEYWORD_INFO("phase"),
                                KEYWORD_INFO("ratio"),
                                KEYWORD_INFO("open"),
                                KEYWORD_INFO("ior"),

                                /* Light Types and options. */
                                KEYWORD_INFO("area_light"),
                                KEYWORD_INFO("looks_like"),
                                KEYWORD_INFO("fade_power"),
                                KEYWORD_INFO("tightness"),
                                KEYWORD_INFO("spotlight"),
                                KEYWORD_INFO("parallel"),
                                KEYWORD_INFO("point_at"),
                                KEYWORD_INFO("falloff"),
                                KEYWORD_INFO("radius"),

                                /* Camera Types and options. */
                                KEYWORD_INFO("omni_directional_stereo"),
                                KEYWORD_INFO("lambert_cylindrical"),
                                KEYWORD_INFO("miller_cylindrical"),
                                KEYWORD_INFO("lambert_azimuthal"),
                                KEYWORD_INFO("ultra_wide_angle"),
                                KEYWORD_INFO("camera_direction"),
                                KEYWORD_INFO("camera_location "),
                                KEYWORD_INFO("van_der_grinten"),
                                KEYWORD_INFO("aitoff_hammer"),
                                KEYWORD_INFO("smyth_craster"),
                                KEYWORD_INFO("orthographic"),
                                KEYWORD_INFO("camera_right"),
                                KEYWORD_INFO("blur_samples"),
                                KEYWORD_INFO("plate_carree"),
                                KEYWORD_INFO("camera_type"),
                                KEYWORD_INFO("perspective"),
                                KEYWORD_INFO("mesh_camera"),
                                KEYWORD_INFO("focal_point"),
                                KEYWORD_INFO("balthasart"),
                                KEYWORD_INFO("confidence"),
                                KEYWORD_INFO("parallaxe"),
                                KEYWORD_INFO("hobo_dyer"),
                                KEYWORD_INFO("camera_up"),
                                KEYWORD_INFO("panoramic"),
                                KEYWORD_INFO("eckert_vi"),
                                KEYWORD_INFO("eckert_iv"),
                                KEYWORD_INFO("mollweide"),
                                KEYWORD_INFO("aperture"),
                                KEYWORD_INFO("behrmann"),
                                KEYWORD_INFO("variance"),
                                KEYWORD_INFO("stereo"),
                                KEYWORD_INFO("icosa"),
                                KEYWORD_INFO("tetra"),
                                KEYWORD_INFO("octa"),
                                KEYWORD_INFO("mercator"),
                                KEYWORD_INFO("omnimax"),
                                KEYWORD_INFO("fisheye"),
                                KEYWORD_INFO("edwards"),
                                KEYWORD_INFO("peters"),
                                KEYWORD_INFO("gall")};

  i = find_keyword_length(keywords, string);

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_pov_find_bool(const char *string)
{
  int i;

  /* Keep aligned args for readability. */

  /* Built-in Constants. */
  constexpr keyword_info keywords[]{KEYWORD_INFO("unofficial"),
                                KEYWORD_INFO("false"),
                                KEYWORD_INFO("no"),
                                KEYWORD_INFO("off"),
                                KEYWORD_INFO("true"),
                                KEYWORD_INFO("yes"),
                                KEYWORD_INFO("on"),
                                KEYWORD_INFO("pi"),
                                KEYWORD_INFO("tau"),
                                /* Encodings. */
                                KEYWORD_INFO("sint16be"),
                                KEYWORD_INFO("sint16le"),
                                KEYWORD_INFO("sint32be"),
                                KEYWORD_INFO("sint32le"),
                                KEYWORD_INFO("uint16be"),
                                KEYWORD_INFO("uint16le"),
                                KEYWORD_INFO("bt2020"),
                                KEYWORD_INFO("bt709"),
                                KEYWORD_INFO("sint8"),
                                KEYWORD_INFO("uint8"),
                                KEYWORD_INFO("ascii"),
                                KEYWORD_INFO("utf8"),
                                /* File-types. */
                                KEYWORD_INFO("tiff"),
                                KEYWORD_INFO("df3"),
                                KEYWORD_INFO("exr"),
                                KEYWORD_INFO("gif"),
                                KEYWORD_INFO("hdr"),
                                KEYWORD_INFO("iff"),
                                KEYWORD_INFO("jpeg"),
                                KEYWORD_INFO("pgm"),
                                KEYWORD_INFO("png"),
                                KEYWORD_INFO("ppm"),
                                KEYWORD_INFO("sys"),
                                KEYWORD_INFO("tga"),
                                KEYWORD_INFO("ttf")};

  i = find_keyword_length(keywords, string);
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

void ED_text_format_register_pov()
{
  static TextFormatType tft = {nullptr};
  static const char *ext[] = {"pov", "inc", "mcr", "mac", nullptr};

  tft.format_identifier = txtfmt_pov_format_identifier;
  tft.format_line = txtfmt_pov_format_line;
  tft.ext = ext;
  tft.comment_line = "//";

  ED_text_format_register(&tft);
}
