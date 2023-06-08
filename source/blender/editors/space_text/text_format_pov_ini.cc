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

/* *** POV INI Keywords (for format_line) *** */

/**
 * Checks the specified source string for a POV INI keyword (minus boolean & 'nil').
 * This name must start at the beginning of the source string and must be
 * followed by a non-identifier (see #text_check_identifier(char)) or null char.
 *
 * If a keyword is found, the length of the matching word is returned.
 * Otherwise, -1 is returned.
 *
 * See:
 * http://www.povray.org/documentation/view/3.7.0/212/
 */
static int txtfmt_ini_find_keyword(const char *string)
{
  int i;

  /* Keep aligned args for readability. */

  /* Language Directives */
  constexpr keyword_info keywords[]{
      KEYWORD_INFO("deprecated"), KEYWORD_INFO("statistics"), KEYWORD_INFO("declare"),
      KEYWORD_INFO("default"),    KEYWORD_INFO("version"),    KEYWORD_INFO("warning"),
      KEYWORD_INFO("include"),    KEYWORD_INFO("fclose"),     KEYWORD_INFO("ifndef"),
      KEYWORD_INFO("append"),     KEYWORD_INFO("elseif"),     KEYWORD_INFO("debug"),
      KEYWORD_INFO("error"),      KEYWORD_INFO("fopen"),      KEYWORD_INFO("ifdef"),
      KEYWORD_INFO("local"),      KEYWORD_INFO("macro"),      KEYWORD_INFO("range"),
      KEYWORD_INFO("render"),     KEYWORD_INFO("break"),      KEYWORD_INFO("switch"),
      KEYWORD_INFO("undef"),      KEYWORD_INFO("while"),      KEYWORD_INFO("write"),
      KEYWORD_INFO("case"),       KEYWORD_INFO("else"),       KEYWORD_INFO("read"),
      KEYWORD_INFO("end"),        KEYWORD_INFO("for"),        KEYWORD_INFO("if"),

      KEYWORD_INFO("I"),          KEYWORD_INFO("S"),          KEYWORD_INFO("A"),
      KEYWORD_INFO("Q"),          KEYWORD_INFO("U"),          KEYWORD_INFO("F"),
      KEYWORD_INFO("C"),          KEYWORD_INFO("N"),          KEYWORD_INFO("P"),
      KEYWORD_INFO("T")};

  i = find_keyword_length(keywords, string);

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_ini_find_reserved(const char *string)
{
  int i;

  /* Keep aligned args for readability. */

  /* POV-Ray Built-in INI Variables
   * list is from...
   * http://www.povray.org/documentation/view/3.7.0/212/
   */
  constexpr keyword_info keywords[]{KEYWORD_INFO("RenderCompleteSoundEnabled"),
                                    KEYWORD_INFO("Create_Continue_Trace_Log"),
                                    KEYWORD_INFO("ParseErrorSoundEnabled"),
                                    KEYWORD_INFO("RenderErrorSoundEnabled"),
                                    KEYWORD_INFO("HideWhenMainMinimized"),
                                    KEYWORD_INFO("Antialias_Confidence"),
                                    KEYWORD_INFO("RenderCompleteSound"),
                                    KEYWORD_INFO("ParseErrorSound"),
                                    KEYWORD_INFO("RenderErrorSound"),
                                    KEYWORD_INFO("UseExtensions"),
                                    KEYWORD_INFO("ReadWriteSourceDir"),
                                    KEYWORD_INFO("NormalPositionLeft"),
                                    KEYWORD_INFO("NormalPositionTop"),
                                    KEYWORD_INFO("NormalPositionRight"),
                                    KEYWORD_INFO("NormalPositionBottom"),
                                    KEYWORD_INFO("Pre_Scene_Command"),
                                    KEYWORD_INFO("Pre_Frame_Command"),
                                    KEYWORD_INFO("Post_Scene_Command"),
                                    KEYWORD_INFO("Post_Frame_Command"),
                                    KEYWORD_INFO("User_Abort_Command"),
                                    KEYWORD_INFO("Fatal_Error_Command"),
                                    KEYWORD_INFO("NormalPositionX"),
                                    KEYWORD_INFO("NormalPositionY"),
                                    KEYWORD_INFO("Pre_Scene_Return"),
                                    KEYWORD_INFO("Pre_Frame_Return"),
                                    KEYWORD_INFO("Post_Scene_Return"),
                                    KEYWORD_INFO("Post_Frame_Return"),
                                    KEYWORD_INFO("User_Abort_Return"),
                                    KEYWORD_INFO("Fatal_Error_Return"),
                                    KEYWORD_INFO("Antialias_Threshold"),
                                    KEYWORD_INFO("Antialias_Gamma"),
                                    KEYWORD_INFO("Antialias_Depth"),
                                    KEYWORD_INFO("input_file_name"),
                                    KEYWORD_INFO("Subset_Start_Frame"),
                                    KEYWORD_INFO("Subset_End_Frame"),
                                    KEYWORD_INFO("UseToolbar"),
                                    KEYWORD_INFO("UseTooltips"),
                                    KEYWORD_INFO("Frame_Step"),
                                    KEYWORD_INFO("Cyclic_Animation"),
                                    KEYWORD_INFO("Field_Render"),
                                    KEYWORD_INFO("Odd_Field"),
                                    KEYWORD_INFO("final_clock"),
                                    KEYWORD_INFO("final_frame"),
                                    KEYWORD_INFO("frame_number"),
                                    KEYWORD_INFO("initial_clock"),
                                    KEYWORD_INFO("initial_frame"),
                                    KEYWORD_INFO("image_height"),
                                    KEYWORD_INFO("image_width"),
                                    KEYWORD_INFO("Start_Column"),
                                    KEYWORD_INFO("Start_Row"),
                                    KEYWORD_INFO("End_Column"),
                                    KEYWORD_INFO("End_Row"),
                                    KEYWORD_INFO("Test_Abort_Count"),
                                    KEYWORD_INFO("Test_Abort"),
                                    KEYWORD_INFO("Continue_Trace"),
                                    KEYWORD_INFO("Bounding_Method"),
                                    KEYWORD_INFO("Create_Ini"),
                                    KEYWORD_INFO("Display_Gamma"),

                                    KEYWORD_INFO("Display"),
                                    KEYWORD_INFO("Version"),
                                    KEYWORD_INFO("Pause_When_Done"),
                                    KEYWORD_INFO("Verbose"),
                                    KEYWORD_INFO("Preview_Start_Size"),
                                    KEYWORD_INFO("Preview_End_Size"),
                                    KEYWORD_INFO("Output_to_File"),
                                    KEYWORD_INFO("Input_File_Name"),
                                    KEYWORD_INFO("Output_File_Name"),
                                    KEYWORD_INFO("Output_File_Type"),
                                    KEYWORD_INFO("Output_Alpha"),
                                    KEYWORD_INFO("Bits_Per_Color"),
                                    KEYWORD_INFO("Compression"),
                                    KEYWORD_INFO("Dither_Method"),
                                    KEYWORD_INFO("Include_Header"),
                                    KEYWORD_INFO("Library_Path"),
                                    KEYWORD_INFO("Debug_Console"),
                                    KEYWORD_INFO("Fatal_Console"),
                                    KEYWORD_INFO("Render_Console"),
                                    KEYWORD_INFO("Statistic_Console"),
                                    KEYWORD_INFO("Warning_Console"),
                                    KEYWORD_INFO("Warning_Level"),
                                    KEYWORD_INFO("All_Console"),
                                    KEYWORD_INFO("Debug_File"),
                                    KEYWORD_INFO("Fatal_File"),
                                    KEYWORD_INFO("Render_File"),
                                    KEYWORD_INFO("Statistic_File"),
                                    KEYWORD_INFO("Warning_File"),
                                    KEYWORD_INFO("All_File"),
                                    KEYWORD_INFO("Quality"),
                                    KEYWORD_INFO("Bounding_Threshold"),
                                    KEYWORD_INFO("Bounding"),
                                    KEYWORD_INFO("Light_Buffer"),
                                    KEYWORD_INFO("Vista_Buffer"),
                                    KEYWORD_INFO("Remove_Bounds"),
                                    KEYWORD_INFO("Split_Unions"),
                                    KEYWORD_INFO("Antialias"),
                                    KEYWORD_INFO("Glare_Desaturation"),
                                    KEYWORD_INFO("Sampling_Method"),
                                    KEYWORD_INFO("Stochastic_Seed"),
                                    KEYWORD_INFO("Jitter_Amount"),
                                    KEYWORD_INFO("Jitter"),
                                    KEYWORD_INFO("Antialias_Depth"),
                                    KEYWORD_INFO("CheckNewVersion"),
                                    KEYWORD_INFO("RunCount"),
                                    KEYWORD_INFO("CommandLine"),
                                    KEYWORD_INFO("TextColour"),
                                    KEYWORD_INFO("WarningColour"),
                                    KEYWORD_INFO("ErrorColour"),
                                    KEYWORD_INFO("BackgroundColour"),

                                    KEYWORD_INFO("DropToEditor"),
                                    KEYWORD_INFO("LastRenderName"),
                                    KEYWORD_INFO("LastRenderPath"),
                                    KEYWORD_INFO("LastQueuePath"),
                                    KEYWORD_INFO("SecondaryINISection"),
                                    KEYWORD_INFO("BetaVersionNo64"),
                                    KEYWORD_INFO("LastBitmapName"),
                                    KEYWORD_INFO("LastBitmapPath"),
                                    KEYWORD_INFO("LastINIPath"),
                                    KEYWORD_INFO("SecondaryINIFile"),
                                    KEYWORD_INFO("BackgroundFile"),
                                    KEYWORD_INFO("SaveSettingsOnExit"),
                                    KEYWORD_INFO("TileBackground"),
                                    KEYWORD_INFO("HideNewUserHelp"),
                                    KEYWORD_INFO("SendSystemInfo"),
                                    KEYWORD_INFO("ItsAboutTime"),
                                    KEYWORD_INFO("LastPath"),
                                    KEYWORD_INFO("Band0Width"),
                                    KEYWORD_INFO("Band1Width"),
                                    KEYWORD_INFO("Band2Width"),
                                    KEYWORD_INFO("Band3Width"),
                                    KEYWORD_INFO("Band4Width"),
                                    KEYWORD_INFO("ShowCmd"),
                                    KEYWORD_INFO("Transparency"),
                                    KEYWORD_INFO("Use8BitMode"),
                                    KEYWORD_INFO("MakeActive"),
                                    KEYWORD_INFO("KeepAboveMain"),
                                    KEYWORD_INFO("AutoClose"),
                                    KEYWORD_INFO("PreserveBitmap"),
                                    KEYWORD_INFO("FontSize"),
                                    KEYWORD_INFO("FontWeight"),
                                    KEYWORD_INFO("KeepMessages"),
                                    KEYWORD_INFO("AlertSound"),
                                    KEYWORD_INFO("Completion"),
                                    KEYWORD_INFO("Priority"),
                                    KEYWORD_INFO("DutyCycle"),
                                    KEYWORD_INFO("AlertOnCompletion"),
                                    KEYWORD_INFO("AutoRender"),
                                    KEYWORD_INFO("PreventSleep"),
                                    KEYWORD_INFO("NoShelloutWait"),
                                    KEYWORD_INFO("SystemNoActive"),
                                    KEYWORD_INFO("NoShellOuts"),
                                    KEYWORD_INFO("VideoSource"),
                                    KEYWORD_INFO("SceneFile"),
                                    KEYWORD_INFO("OutputFile"),
                                    KEYWORD_INFO("IniOutputFile"),
                                    KEYWORD_INFO("CurrentDirectory"),
                                    KEYWORD_INFO("SourceFile"),
                                    KEYWORD_INFO("Rendering"),
                                    KEYWORD_INFO("RenderwinClose"),
                                    KEYWORD_INFO("Append_File"),
                                    KEYWORD_INFO("Warning Level"),
                                    KEYWORD_INFO("clock_delta"),
                                    KEYWORD_INFO("clock_on"),
                                    KEYWORD_INFO("clock"),
                                    KEYWORD_INFO("Height"),
                                    KEYWORD_INFO("Width"),
                                    KEYWORD_INFO("Dither"),
                                    KEYWORD_INFO("Flags"),
                                    KEYWORD_INFO("Font"),

                                    /* File-types. */
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
                                    KEYWORD_INFO("tiff"),
                                    /* Encodings. */
                                    KEYWORD_INFO("ascii"),
                                    KEYWORD_INFO("utf8"),
                                    KEYWORD_INFO("uint8"),
                                    KEYWORD_INFO("uint16be"),
                                    KEYWORD_INFO("uint16le"),
                                    KEYWORD_INFO("sint8"),
                                    KEYWORD_INFO("sint16be"),
                                    KEYWORD_INFO("sint16le"),
                                    KEYWORD_INFO("sint32be"),
                                    KEYWORD_INFO("sint32le")};

  i = find_keyword_length(keywords, string);

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_ini_find_bool(const char *string)
{
  int i;

  /* Keep aligned args for readability. */

  /* Built-in Constants */
  constexpr keyword_info keywords[]{
      KEYWORD_INFO("false"),
      KEYWORD_INFO("no"),
      KEYWORD_INFO("off"),
      KEYWORD_INFO("true"),
      KEYWORD_INFO("yes"),
      KEYWORD_INFO("on"),
      KEYWORD_INFO("pi"),
      KEYWORD_INFO("tau"),
      KEYWORD_INFO("%o"),
      KEYWORD_INFO("%s"),
      KEYWORD_INFO("%n"),
      KEYWORD_INFO("%k"),
      KEYWORD_INFO("%h"),
      KEYWORD_INFO("%w"),
  };

  i = find_keyword_length(keywords, string);

  /* If next source char is an identifier (eg. 'i' in "Nonetheless") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static char txtfmt_pov_ini_format_identifier(const char *str)
{
  char fmt;
  if (txtfmt_ini_find_keyword(str) != -1) {
    fmt = FMT_TYPE_KEYWORD;
  }
  else if (txtfmt_ini_find_reserved(str) != -1) {
    fmt = FMT_TYPE_RESERVED;
  }
  else {
    fmt = FMT_TYPE_DEFAULT;
  }
  return fmt;
}

static void txtfmt_pov_ini_format_line(SpaceText *st, TextLine *line, const bool do_next)
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
      /* Multi-line comments */
      if (cont & FMT_CONT_COMMENT_C) {
        if (*str == ']' && *(str + 1) == ']') {
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
      /* Multi-line comments not supported */
      /* Single line comment */
      if (*str == ';') {
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
      else if (prev != FMT_TYPE_DEFAULT && (i = txtfmt_ini_find_bool(str)) != -1) {
        if (i > 0) {
          text_format_fill_ascii(&str, &fmt, FMT_TYPE_NUMERAL, i);
        }
        else {
          str += BLI_str_utf8_size_safe(str) - 1;
          *fmt = FMT_TYPE_DEFAULT;
        }
      }
      /* Punctuation */
      else if ((*str != '#') && text_check_delim(*str)) {
        *fmt = FMT_TYPE_SYMBOL;
      }
      /* Identifiers and other text (no previous white-space/delimiters so text continues). */
      else if (prev == FMT_TYPE_DEFAULT) {
        str += BLI_str_utf8_size_safe(str) - 1;
        *fmt = FMT_TYPE_DEFAULT;
      }
      /* Not white-space, a digit, punctuation, or continuing text.
       * Must be new, check for special words */
      else {
        /* Keep aligned arguments for readability. */
        /* clang-format off */

        /* Special vars(v) or built-in keywords(b) */
        /* keep in sync with `txtfmt_ini_format_identifier()`. */
        if        ((i = txtfmt_ini_find_keyword(str))  != -1) { prev = FMT_TYPE_KEYWORD;
        } else if ((i = txtfmt_ini_find_reserved(str)) != -1) { prev = FMT_TYPE_RESERVED;
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
    txtfmt_pov_ini_format_line(st, line->next, do_next);
  }

  flatten_string_free(&fs);
}

void ED_text_format_register_pov_ini()
{
  static TextFormatType tft = {nullptr};
  static const char *ext[] = {"ini", nullptr};

  tft.format_identifier = txtfmt_pov_ini_format_identifier;
  tft.format_line = txtfmt_pov_ini_format_line;
  tft.ext = ext;
  tft.comment_line = "//";

  ED_text_format_register(&tft);
}
