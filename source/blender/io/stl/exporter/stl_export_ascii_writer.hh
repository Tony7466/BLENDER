/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup stl
 */

#pragma once

#include <fstream>

#include "BLI_fileops.hh"
#include "BLI_utility_mixins.hh"

/* SEP macro from BLI path utils clashes with SEP symbol in fmt headers. */
#undef SEP
#include <fmt/format.h>

#include "stl_export_writer.hh"

namespace blender::io::stl {

class ASCIIFileWriter : public FileWriter, NonCopyable {
 private:
  blender::fstream file_;

 public:
  explicit ASCIIFileWriter(const char *filepath);
  ~ASCIIFileWriter() override;
  void write_triangle(const Triangle &t) override;
};

ASCIIFileWriter::ASCIIFileWriter(const char *filepath) : file_(filepath)
{
  file_ << "solid \n";
}

void ASCIIFileWriter::write_triangle(const Triangle &t)
{
  file_ << fmt::format(
      "facet normal {} {} {}\n"
      "\touter loop\n"
      "\t\tvertex {} {} {}\n"
      "\t\tvertex {} {} {}\n"
      "\t\tvertex {} {} {}\n"
      "\tendloop\n"
      "endfacet\n",

      t.normal.x,
      t.normal.y,
      t.normal.z,
      t.vertices[0].x,
      t.vertices[0].y,
      t.vertices[0].z,
      t.vertices[1].x,
      t.vertices[1].y,
      t.vertices[1].z,
      t.vertices[2].x,
      t.vertices[2].y,
      t.vertices[2].z);
}

ASCIIFileWriter::~ASCIIFileWriter()
{
  file_ << "endsolid \n";
}

}  // namespace blender::io::stl
