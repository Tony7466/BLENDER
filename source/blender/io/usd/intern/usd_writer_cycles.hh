
#ifndef BLENDER_USD_WRITER_CYCLES_H
#define BLENDER_USD_WRITER_CYCLES_H

#include "DNA_material_types.h"
#include "DNA_node_types.h"
#include <pxr/usd/usdShade/material.h>


namespace blender::io::usd {

void create_usd_cycles_material(pxr::UsdStageRefPtr a_stage,
                                Material *material,
                                pxr::UsdShadeMaterial &usd_material,
                                const USDExportParams &export_params);

void create_usd_cycles_material(pxr::UsdStageRefPtr a_stage,
                                bNodeTree *ntree,
                                pxr::UsdShadeMaterial &usd_material,
                                const USDExportParams &export_params);

pxr::UsdShadeShader create_cycles_shader_node(pxr::UsdStageRefPtr a_stage,
                                              pxr::SdfPath &shaderPath,
                                              bNode *node,
                                              const USDExportParams &export_params);

}; // namespace namespace blender::io::usd


#endif  // BLENDER_USD_WRITER_CYCLES_H
