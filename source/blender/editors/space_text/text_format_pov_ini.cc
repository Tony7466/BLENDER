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

/**
 * POV INI keyword (minus boolean & 'nil')
 * See:
 * http://www.povray.org/documentation/view/3.7.0/212/
 */

/* Language Directives */

static StringRef text_format_pov_ini_literals_keyword[]{
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

    StringRef("A"),
    StringRef("C"),
    StringRef("F"),
    StringRef("I"),
    StringRef("N"),
    StringRef("P"),
    StringRef("Q"),
    StringRef("S"),
    StringRef("T"),
    StringRef("U"),
    /* clang-format on */
};

/**
 * POV-Ray Built-in INI Variables
 * list is from...
 * http://www.povray.org/documentation/view/3.7.0/212/
 */
static StringRef text_format_pov_ini_literals_reserved[]{
    /* clang-format off */
    StringRef("AlertOnCompletion"),
    StringRef("AlertSound"),
    StringRef("All_Console"),
    StringRef("All_File"),
    StringRef("Antialias"),
    StringRef("Antialias_Confidence"),
    StringRef("Antialias_Depth"),
    StringRef("Antialias_Depth"),
    StringRef("Antialias_Gamma"),
    StringRef("Antialias_Threshold"),
    StringRef("Append_File"),
    StringRef("AutoClose"),
    StringRef("AutoRender"),
    StringRef("BackgroundColour"),
    StringRef("BackgroundFile"),
    StringRef("Band0Width"),
    StringRef("Band1Width"),
    StringRef("Band2Width"),
    StringRef("Band3Width"),
    StringRef("Band4Width"),
    StringRef("BetaVersionNo64"),
    StringRef("Bits_Per_Color"),
    StringRef("Bounding"),
    StringRef("Bounding_Method"),
    StringRef("Bounding_Threshold"),
    StringRef("CheckNewVersion"),
    StringRef("CommandLine"),
    StringRef("Completion"),
    StringRef("Compression"),
    StringRef("Continue_Trace"),
    StringRef("Create_Continue_Trace_Log"),
    StringRef("Create_Ini"),
    StringRef("CurrentDirectory"),
    StringRef("Cyclic_Animation"),
    StringRef("Debug_Console"),
    StringRef("Debug_File"),
    StringRef("Display"),
    StringRef("Display_Gamma"),
    StringRef("Dither"),
    StringRef("Dither_Method"),
    StringRef("DropToEditor"),
    StringRef("DutyCycle"),
    StringRef("End_Column"),
    StringRef("End_Row"),
    StringRef("ErrorColour"),
    StringRef("Fatal_Console"),
    StringRef("Fatal_Error_Command"),
    StringRef("Fatal_Error_Return"),
    StringRef("Fatal_File"),
    StringRef("Field_Render"),
    StringRef("Flags"),
    StringRef("Font"),
    StringRef("FontSize"),
    StringRef("FontWeight"),
    StringRef("Frame_Step"),
    StringRef("Glare_Desaturation"),
    StringRef("Height"),
    StringRef("HideNewUserHelp"),
    StringRef("HideWhenMainMinimized"),
    StringRef("Include_Header"),
    StringRef("IniOutputFile"),
    StringRef("Input_File_Name"),
    StringRef("ItsAboutTime"),
    StringRef("Jitter"),
    StringRef("Jitter_Amount"),
    StringRef("KeepAboveMain"),
    StringRef("KeepMessages"),
    StringRef("LastBitmapName"),
    StringRef("LastBitmapPath"),
    StringRef("LastINIPath"),
    StringRef("LastPath"),
    StringRef("LastQueuePath"),
    StringRef("LastRenderName"),
    StringRef("LastRenderPath"),
    StringRef("Library_Path"),
    StringRef("Light_Buffer"),
    StringRef("MakeActive"),
    StringRef("NoShellOuts"),
    StringRef("NoShelloutWait"),
    StringRef("NormalPositionBottom"),
    StringRef("NormalPositionLeft"),
    StringRef("NormalPositionRight"),
    StringRef("NormalPositionTop"),
    StringRef("NormalPositionX"),
    StringRef("NormalPositionY"),
    StringRef("Odd_Field"),
    StringRef("OutputFile"),
    StringRef("Output_Alpha"),
    StringRef("Output_File_Name"),
    StringRef("Output_File_Type"),
    StringRef("Output_to_File"),
    StringRef("ParseErrorSound"),
    StringRef("ParseErrorSoundEnabled"),
    StringRef("Pause_When_Done"),
    StringRef("Post_Frame_Command"),
    StringRef("Post_Frame_Return"),
    StringRef("Post_Scene_Command"),
    StringRef("Post_Scene_Return"),
    StringRef("Pre_Frame_Command"),
    StringRef("Pre_Frame_Return"),
    StringRef("Pre_Scene_Command"),
    StringRef("Pre_Scene_Return"),
    StringRef("PreserveBitmap"),
    StringRef("PreventSleep"),
    StringRef("Preview_End_Size"),
    StringRef("Preview_Start_Size"),
    StringRef("Priority"),
    StringRef("Quality"),
    StringRef("ReadWriteSourceDir"),
    StringRef("Remove_Bounds"),
    StringRef("RenderCompleteSound"),
    StringRef("RenderCompleteSoundEnabled"),
    StringRef("RenderErrorSound"),
    StringRef("RenderErrorSoundEnabled"),
    StringRef("Render_Console"),
    StringRef("Render_File"),
    StringRef("Rendering"),
    StringRef("RenderwinClose"),
    StringRef("RunCount"),
    StringRef("Sampling_Method"),
    StringRef("SaveSettingsOnExit"),
    StringRef("SceneFile"),
    StringRef("SecondaryINIFile"),
    StringRef("SecondaryINISection"),
    StringRef("SendSystemInfo"),
    StringRef("ShowCmd"),
    StringRef("SourceFile"),
    StringRef("Split_Unions"),
    StringRef("Start_Column"),
    StringRef("Start_Row"),
    StringRef("Statistic_Console"),
    StringRef("Statistic_File"),
    StringRef("Stochastic_Seed"),
    StringRef("Subset_End_Frame"),
    StringRef("Subset_Start_Frame"),
    StringRef("SystemNoActive"),
    StringRef("Test_Abort"),
    StringRef("Test_Abort_Count"),
    StringRef("TextColour"),
    StringRef("TileBackground"),
    StringRef("Transparency"),
    StringRef("Use8BitMode"),
    StringRef("UseExtensions"),
    StringRef("UseToolbar"),
    StringRef("UseTooltips"),
    StringRef("User_Abort_Command"),
    StringRef("User_Abort_Return"),
    StringRef("Verbose"),
    StringRef("Version"),
    StringRef("VideoSource"),
    StringRef("Vista_Buffer"),
    StringRef("Warning Level"),
    StringRef("WarningColour"),
    StringRef("Warning_Console"),
    StringRef("Warning_File"),
    StringRef("Warning_Level"),
    StringRef("Width"),
    StringRef("clock"),
    StringRef("clock_delta"),
    StringRef("clock_on"),
    StringRef("final_clock"),
    StringRef("final_frame"),
    StringRef("frame_number"),
    StringRef("image_height"),
    StringRef("image_width"),
    StringRef("initial_clock"),
    StringRef("initial_frame"),
    StringRef("input_file_name"),

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
    /* Encodings. */
    StringRef("ascii"),
    StringRef("sint16be"),
    StringRef("sint16le"),
    StringRef("sint32be"),
    StringRef("sint32le"),
    StringRef("sint8"),
    StringRef("uint16be"),
    StringRef("uint16le"),
    StringRef("uint8"),
    StringRef("utf8"),
    /* clang-format on */
};

/** POV INI Built-in Constants */
static StringRef text_format_pov_ini_literals_bool[]{
    /* clang-format off */
    StringRef("%h"),
    StringRef("%k"),
    StringRef("%n"),
    StringRef("%o"),
    StringRef("%s"),
    StringRef("%w"),
    StringRef("false"),
    StringRef("no"),
    StringRef("off"),
    StringRef("on"),
    StringRef("pi"),
    StringRef("tau"),
    StringRef("true"),
    StringRef("yes"),
    /* clang-format on */
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Local Functions (for #TextFormatType::format_line)
 * \{ */

static int txtfmt_ini_find_keyword(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_pov_ini_literals_keyword, string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_ini_find_reserved(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_pov_ini_literals_reserved, string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  return (i == 0 || text_check_identifier(string[i])) ? -1 : i;
}

static int txtfmt_ini_find_bool(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_pov_ini_literals_bool, string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

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

/** \} */

/* -------------------------------------------------------------------- */
/** \name Format Line Implementation (#TextFormatType::format_line)
 * \{ */

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

/** \} */

/* -------------------------------------------------------------------- */
/** \name Registration
 * \{ */

void ED_text_format_register_pov_ini()
{
  static TextFormatType tft = {nullptr};
  static const char *ext[] = {"ini", nullptr};

  tft.format_identifier = txtfmt_pov_ini_format_identifier;
  tft.format_line = txtfmt_pov_ini_format_line;
  tft.ext = ext;
  tft.comment_line = "//";

  ED_text_format_register(&tft);

  text_format_string_literals_sort_for_lookup(text_format_pov_ini_literals_keyword);
  text_format_string_literals_sort_for_lookup(text_format_pov_ini_literals_reserved);
  text_format_string_literals_sort_for_lookup(text_format_pov_ini_literals_bool);
}

/** \} */
