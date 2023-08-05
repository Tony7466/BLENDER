/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup RNA
 */

#include <cstdlib>

#include "BLI_utildefines.h"

#include "RNA_access.h"
#include "RNA_define.h"

#include "rna_internal.h"

#ifdef RNA_RUNTIME

static int get_my_value(PointerRNA * /*ptr*/)
{
  return 55;
}

#else

static void rna_def_dummy(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "Dummy", nullptr);
  RNA_def_struct_ui_text(srna, "Dummy Name", "Dummy Description");

  prop = RNA_def_property(srna, "my_value", PROP_INT, PROP_NONE);
  RNA_def_property_int_funcs(prop, "get_my_value", nullptr, nullptr);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
}

void RNA_def_dummy(BlenderRNA *brna)
{
  rna_def_dummy(brna);
}

#endif
