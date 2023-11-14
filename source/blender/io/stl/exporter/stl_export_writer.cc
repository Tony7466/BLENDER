/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup stl
 */

#include <memory>
#include <stdexcept>

#include "stl_export_ascii_writer.hh"
#include "stl_export_binary_writer.hh"

namespace blender::io::stl {

std::unique_ptr<FileWriter> create_writer(const char *filepath, FileWriter::Type type)
{
  if (type == FileWriter::Type::ASCII) {
    return std::make_unique<ASCIIFileWriter>(filepath);
  }
  if (type == FileWriter::Type::BINARY) {
    return std::make_unique<BinaryFileWriter>(filepath);
  }
  throw std::runtime_error("Unknown STL export writer type");
}

}  // namespace blender::io::stl
