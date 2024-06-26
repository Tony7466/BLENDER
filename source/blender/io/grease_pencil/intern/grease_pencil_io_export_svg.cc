/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "grease_pencil_io.hh"

#include "pugixml.hpp"

/** \file
 * \ingroup bgrease_pencil
 */

namespace blender::io::grease_pencil {

class SVGExporter : public GreasePencilExporter {
 public:
  using GreasePencilExporter::GreasePencilExporter;

  pugi::xml_document main_doc_;
  pugi::xml_node main_node_;
  pugi::xml_node frame_node_;

  bool write(Scene &scene, StringRefNull filepath);

  void create_document_header();
};

void SVGExporter::create_document_header()
{
  /* Add a custom document declaration node. */
  pugi::xml_node decl = main_doc_.prepend_child(pugi::node_declaration);
  decl.append_attribute("version") = "1.0";
  decl.append_attribute("encoding") = "UTF-8";

  pugi::xml_node comment = main_doc_.append_child(pugi::node_comment);
  char txt[128];
  SNPRINTF(txt, " Generator: Blender, %s - %s ", SVG_EXPORTER_NAME, SVG_EXPORTER_VERSION);
  comment.set_value(txt);

  pugi::xml_node doctype = main_doc_.append_child(pugi::node_doctype);
  doctype.set_value(
      "svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" "
      "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\"");

  main_node_ = main_doc_.append_child("svg");
  main_node_.append_attribute("version").set_value("1.0");
  main_node_.append_attribute("x").set_value("0px");
  main_node_.append_attribute("y").set_value("0px");
  main_node_.append_attribute("xmlns").set_value("http://www.w3.org/2000/svg");

  std::string width;
  std::string height;

  width = std::to_string(render_x_);
  height = std::to_string(render_y_);

  main_node_.append_attribute("width").set_value((width + "px").c_str());
  main_node_.append_attribute("height").set_value((height + "px").c_str());
  std::string viewbox = "0 0 " + width + " " + height;
  main_node_.append_attribute("viewBox").set_value(viewbox.c_str());
}

bool SVGExporter::write(Scene &scene, StringRefNull filepath)
{
  this->prepare_camera_params(scene, false);

  exporter->add_newpage();
  exporter->add_body();
  return true;
}

bool export_svg(const IOContext &context,
                const ExportParams &params,
                Scene &scene,
                StringRefNull filepath)
{
  SVGExporter exporter(context, params);
  return exporter.write(scene, filepath);
}

}  // namespace blender::io::grease_pencil
