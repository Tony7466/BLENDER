/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "grease_pencil_io_intern.hh"

/** \file
 * \ingroup bgrease_pencil
 */

namespace blender::io::grease_pencil {

class PDFExporter : public GreasePencilExporter {
 public:
  using GreasePencilExporter::GreasePencilExporter;

  bool write(Scene &scene, StringRefNull filepath);
};

bool export_pdf(const IOContext &context,
                const ExportParams &params,
                Scene &scene,
                StringRefNull filepath)
{
  PDFExporter exporter(context, params);
  return exporter.write(scene, filepath);
}

}  // namespace blender::io::grease_pencil
