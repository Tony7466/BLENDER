/* SPDX-FileCopyrightText: 2005 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#pragma once

#include "BKE_node.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct bNodeTreeType *ntreeType_Shader;

void register_node_type_sh_custom_group(bNodeType *ntype);

struct bNodeTreeExec *ntreeShaderBeginExecTree(struct bNodeTree *ntree);
void ntreeShaderEndExecTree(struct bNodeTreeExec *exec);

/**
 * Find an output node of the shader tree.
 *
 * \note it will only return output which is NOT in the group, which isn't how
 * render engines works but it's how the GPU shader compilation works. This we
 * can change in the future and make it a generic function, but for now it stays
 * private here.
 */
struct bNode *ntreeShaderOutputNode(struct bNodeTree *ntree, int target);

/**
 * This one needs to work on a local tree.
 */
void ntreeGPUMaterialNodes(struct bNodeTree *localtree, struct GPUMaterial *mat);

struct NodeMathFormulaItem *NOD_math_formula_add_item(struct NodeMathFormula *formula,
                                                      short socket_type,
                                                      const char *name);

bool NOD_math_formula_contains_item(struct NodeMathFormula *formula,
                                    const struct NodeMathFormulaItem *item);
struct NodeMathFormulaItem *NOD_math_formula_get_active_item(struct NodeMathFormula *formula);
void NOD_math_formula_set_active_item(struct NodeMathFormula *formula,
                                      struct NodeMathFormulaItem *item);
struct NodeMathFormulaItem *NOD_math_formula_find_item(struct NodeMathFormula *formula,
                                                       const char *name);
struct NodeMathFormulaItem *NOD_math_formula_add_item(struct NodeMathFormula *formula,
                                                      short socket_type,
                                                      const char *name);
struct NodeMathFormulaItem *NOD_math_formula_insert_item(struct NodeMathFormula *formula,
                                                         short socket_type,
                                                         const char *name,
                                                         int index);
struct NodeMathFormulaItem *NOD_math_formula_add_item_from_socket(
    struct NodeMathFormula *formula,
    const struct bNode *from_node,
    const struct bNodeSocket *from_sock);
struct NodeMathFormulaItem *NOD_math_formula_insert_item_from_socket(
    struct NodeMathFormula *formula,
    const struct bNode *from_node,
    const struct bNodeSocket *from_sock,
    int index);
void NOD_math_formula_remove_item(struct NodeMathFormula *formula,
                                  struct NodeMathFormulaItem *item);
void NOD_math_formula_clear_items(struct NodeMathFormula *formula);
void NOD_math_formula_move_item(struct NodeMathFormula *formula, int from_index, int to_index);
struct bNode *NOD_math_formula_find_node_by_item(struct bNodeTree *ntree,
                                                 const struct NodeMathFormulaItem *item);
void NOD_math_formula_rename_item(const struct bNode *node,
                                  struct NodeMathFormulaItem *dest_item,
                                  const char *new_name);
#ifdef __cplusplus
}
#endif
