/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * The Original Code is written by Rob Haarsma (phase). All rights reserved. */

/** \file
 * \ingroup bke
 *
 * This code parses the Freetype font outline data to chains of Blender's bezier-triples.
 * Additional information can be found at the bottom of this file.
 *
 * Code that uses exotic character maps is present but commented out.
 */

#include <ft2build.h>
#include FT_FREETYPE_H

#include "MEM_guardedalloc.h"

#include "BLF_api.h"

#include "BLI_ghash.h"
#include "BLI_listbase.h"
#include "BLI_string.h"
#include "BLI_string_utf8.h"

#include "BKE_curve.h"
#include "BKE_vfont.h"
#include "BKE_vfontdata.h"

#include "DNA_packedFile_types.h"
#include "DNA_vfont_types.h"

extern const void *builtin_font_data;
extern int builtin_font_size;

VFontData *BKE_vfontdata_from_freetypefont(PackedFile *pf)
{
  int fontid = BLF_load_mem("FTVFont", static_cast<const unsigned char *>(pf->data), pf->size);
  if (fontid == -1) {
    return nullptr;
  }

  FT_Face face = static_cast<FT_Face>(BLF_get_face(fontid));
  if (!face) {
    BLF_unload_id(fontid);
    return nullptr;
  }

  /* allocate blender font */
  VFontData *vfd = static_cast<VFontData *>(MEM_callocN(sizeof(*vfd), "FTVFontData"));

  /* Get the font name. */
  if (face->family_name) {
    STRNCPY(vfd->name, BLF_display_name(fontid));
    BLI_str_utf8_invalid_strip(vfd->name, strlen(vfd->name));
  }

  /* Blender default BFont is not "complete". */
  const bool complete_font = (face->ascender != 0) && (face->descender != 0) &&
                             (face->ascender != face->descender);

  if (complete_font) {
    /* We can get descender as well, but we simple store descender in relation to the ascender.
     * Also note that descender is stored as a negative number. */
    vfd->ascender = float(face->ascender) / (face->ascender - face->descender);
  }
  else {
    vfd->ascender = 0.8f;
    vfd->em_height = 1.0f;
  }

  /* Adjust font size */
  if (face->bbox.yMax != face->bbox.yMin) {
    vfd->scale = float(1.0 / double(face->bbox.yMax - face->bbox.yMin));

    if (complete_font) {
      vfd->em_height = float(face->ascender - face->descender) /
                       (face->bbox.yMax - face->bbox.yMin);
    }
  }
  else {
    vfd->scale = 1.0f / 1000.0f;
  }

  vfd->characters = BLI_ghash_int_new_ex(__func__, 255);

  BLF_unload_id(fontid);

  return vfd;
}

static void *vfontdata_copy_characters_value_cb(const void *src)
{
  return BKE_vfontdata_char_copy(static_cast<const VChar *>(src));
}

VFontData *BKE_vfontdata_copy(const VFontData *vfont_src, const int /*flag*/)
{
  VFontData *vfont_dst = static_cast<VFontData *>(MEM_dupallocN(vfont_src));

  if (vfont_src->characters != nullptr) {
    vfont_dst->characters = BLI_ghash_copy(
        vfont_src->characters, nullptr, vfontdata_copy_characters_value_cb);
  }

  return vfont_dst;
}

VChar *BKE_vfontdata_char_from_freetypefont(VFont *vfont, ulong character)
{
  if (!vfont) {
    return nullptr;
  }

  int font_id;

  if (BKE_vfont_is_builtin(vfont)) {
    font_id = BLF_load_mem(vfont->data->name,
                           static_cast<const unsigned char *>(builtin_font_data),
                           builtin_font_size);
  }
  else {
    font_id = BLF_load_mem(vfont->data->name,
                           static_cast<const unsigned char *>(vfont->temp_pf->data),
                           vfont->temp_pf->size);
  }

  if (font_id == -1) {
    return nullptr;
  }

  VChar *che = (VChar *)MEM_callocN(sizeof(VChar), "objfnt_char");
  che->index = character;
  che->width = BLF_character_to_curves(font_id, character, &che->nurbsbase, vfont->data->scale);

  BLI_ghash_insert(vfont->data->characters, POINTER_FROM_UINT(che->index), che);
  BLF_unload_id(font_id);
  return che;
}

VChar *BKE_vfontdata_char_copy(const VChar *vchar_src)
{
  VChar *vchar_dst = static_cast<VChar *>(MEM_dupallocN(vchar_src));

  BLI_listbase_clear(&vchar_dst->nurbsbase);
  BKE_nurbList_duplicate(&vchar_dst->nurbsbase, &vchar_src->nurbsbase);

  return vchar_dst;
}
