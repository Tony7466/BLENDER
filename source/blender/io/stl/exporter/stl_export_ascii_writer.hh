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
  void write_triangle(const Triangle *t) override;
};

ASCIIFileWriter::ASCIIFileWriter(const char *filepath) : file_(filepath)
{
  file_ << "solid \n";
}

void ASCIIFileWriter::write_triangle(const Triangle *t)
{
  file_ << fmt::format(
      "facet normal {} {} {}\n"
      "\touter loop\n"
      "\t\tvertex {} {} {}\n"
      "\t\tvertex {} {} {}\n"
      "\t\tvertex {} {} {}\n"
      "\tendloop\n"
      "endfacet\n",

      t->normal[0],
      t->normal[1],
      t->normal[2],
      t->vertices[0][0],
      t->vertices[0][1],
      t->vertices[0][2],
      t->vertices[1][0],
      t->vertices[1][1],
      t->vertices[1][2],
      t->vertices[2][0],
      t->vertices[2][1],
      t->vertices[2][2]);
}

ASCIIFileWriter::~ASCIIFileWriter()
{
  file_ << "endsolid \n";
}

}  // namespace blender::io::stl
