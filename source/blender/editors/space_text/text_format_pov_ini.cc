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
 * POV INI keyword (minus boolean & 'nil')
 * See:
 * http://www.povray.org/documentation/view/3.7.0/212/
 */

/* Language Directives */
const char *ini_keyword_text[]{
    "deprecated", "statistics", "declare", "default", "version", "warning", "include", "fclose",
    "ifndef",     "append",     "elseif",  "debug",   "error",   "fopen",   "ifdef",   "local",
    "macro",      "range",      "render",  "break",   "switch",  "undef",   "while",   "write",
    "case",       "else",       "read",    "end",     "for",     "if",

    "I",          "S",          "A",       "Q",       "U",       "F",       "C",       "N",
    "P",          "T",
};

/**
 * POV-Ray Built-in INI Variables
 * list is from...
 * http://www.povray.org/documentation/view/3.7.0/212/
 */
const char *ini_reserved_text[]{
    "RenderCompleteSoundEnabled",
    "Create_Continue_Trace_Log",
    "ParseErrorSoundEnabled",
    "RenderErrorSoundEnabled",
    "HideWhenMainMinimized",
    "Antialias_Confidence",
    "RenderCompleteSound",
    "ParseErrorSound",
    "RenderErrorSound",
    "UseExtensions",
    "ReadWriteSourceDir",
    "NormalPositionLeft",
    "NormalPositionTop",
    "NormalPositionRight",
    "NormalPositionBottom",
    "Pre_Scene_Command",
    "Pre_Frame_Command",
    "Post_Scene_Command",
    "Post_Frame_Command",
    "User_Abort_Command",
    "Fatal_Error_Command",
    "NormalPositionX",
    "NormalPositionY",
    "Pre_Scene_Return",
    "Pre_Frame_Return",
    "Post_Scene_Return",
    "Post_Frame_Return",
    "User_Abort_Return",
    "Fatal_Error_Return",
    "Antialias_Threshold",
    "Antialias_Gamma",
    "Antialias_Depth",
    "input_file_name",
    "Subset_Start_Frame",
    "Subset_End_Frame",
    "UseToolbar",
    "UseTooltips",
    "Frame_Step",
    "Cyclic_Animation",
    "Field_Render",
    "Odd_Field",
    "final_clock",
    "final_frame",
    "frame_number",
    "initial_clock",
    "initial_frame",
    "image_height",
    "image_width",
    "Start_Column",
    "Start_Row",
    "End_Column",
    "End_Row",
    "Test_Abort_Count",
    "Test_Abort",
    "Continue_Trace",
    "Bounding_Method",
    "Create_Ini",
    "Display_Gamma",

    "Display",
    "Version",
    "Pause_When_Done",
    "Verbose",
    "Preview_Start_Size",
    "Preview_End_Size",
    "Output_to_File",
    "Input_File_Name",
    "Output_File_Name",
    "Output_File_Type",
    "Output_Alpha",
    "Bits_Per_Color",
    "Compression",
    "Dither_Method",
    "Include_Header",
    "Library_Path",
    "Debug_Console",
    "Fatal_Console",
    "Render_Console",
    "Statistic_Console",
    "Warning_Console",
    "Warning_Level",
    "All_Console",
    "Debug_File",
    "Fatal_File",
    "Render_File",
    "Statistic_File",
    "Warning_File",
    "All_File",
    "Quality",
    "Bounding_Threshold",
    "Bounding",
    "Light_Buffer",
    "Vista_Buffer",
    "Remove_Bounds",
    "Split_Unions",
    "Antialias",
    "Glare_Desaturation",
    "Sampling_Method",
    "Stochastic_Seed",
    "Jitter_Amount",
    "Jitter",
    "Antialias_Depth",
    "CheckNewVersion",
    "RunCount",
    "CommandLine",
    "TextColour",
    "WarningColour",
    "ErrorColour",
    "BackgroundColour",

    "DropToEditor",
    "LastRenderName",
    "LastRenderPath",
    "LastQueuePath",
    "SecondaryINISection",
    "BetaVersionNo64",
    "LastBitmapName",
    "LastBitmapPath",
    "LastINIPath",
    "SecondaryINIFile",
    "BackgroundFile",
    "SaveSettingsOnExit",
    "TileBackground",
    "HideNewUserHelp",
    "SendSystemInfo",
    "ItsAboutTime",
    "LastPath",
    "Band0Width",
    "Band1Width",
    "Band2Width",
    "Band3Width",
    "Band4Width",
    "ShowCmd",
    "Transparency",
    "Use8BitMode",
    "MakeActive",
    "KeepAboveMain",
    "AutoClose",
    "PreserveBitmap",
    "FontSize",
    "FontWeight",
    "KeepMessages",
    "AlertSound",
    "Completion",
    "Priority",
    "DutyCycle",
    "AlertOnCompletion",
    "AutoRender",
    "PreventSleep",
    "NoShelloutWait",
    "SystemNoActive",
    "NoShellOuts",
    "VideoSource",
    "SceneFile",
    "OutputFile",
    "IniOutputFile",
    "CurrentDirectory",
    "SourceFile",
    "Rendering",
    "RenderwinClose",
    "Append_File",
    "Warning Level",
    "clock_delta",
    "clock_on",
    "clock",
    "Height",
    "Width",
    "Dither",
    "Flags",
    "Font",

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
    /* Encodings. */
    "ascii",
    "utf8",
    "uint8",
    "uint16be",
    "uint16le",
    "sint8",
    "sint16be",
    "sint16le",
    "sint32be",
    "sint32le",
};

/* POV INI Built-in Constants */
const char *ini_bool_text[]{
    "false",
    "no",
    "off",
    "true",
    "yes",
    "on",
    "pi",
    "tau",
    "%o",
    "%s",
    "%n",
    "%k",
    "%h",
    "%w",
};

std::vector<KeywordInfo> ini_keyword{};
std::vector<KeywordInfo> ini_reserved{};
std::vector<KeywordInfo> ini_bool{};

static char txtfmt_pov_ini_format_identifier(const char *str)
{
  char fmt;
  if (find_keyword_length(ini_keyword, str) != -1) {
    fmt = FMT_TYPE_KEYWORD;
  }
  else if (find_keyword_length(ini_reserved, str) != -1) {
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
      else if (prev != FMT_TYPE_DEFAULT && (i = find_keyword_length(ini_bool, str)) != -1) {
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
        if        ((i = find_keyword_length(ini_keyword,str))  != -1) { prev = FMT_TYPE_KEYWORD;
        } else if ((i = find_keyword_length(ini_reserved,str)) != -1) { prev = FMT_TYPE_RESERVED;
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

  fill_keyword_vector(ini_keyword, ini_keyword_text);
  fill_keyword_vector(ini_reserved, ini_reserved_text);
  fill_keyword_vector(ini_bool, ini_bool_text);
}
