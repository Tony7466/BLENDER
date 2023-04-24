/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

/** \file
 * \ingroup bke
 */

#include "BKE_anim_data.h"
#include "BKE_curves.hh"
#include "BKE_customdata.h"
#include "BKE_grease_pencil.h"
#include "BKE_grease_pencil.hh"
#include "BKE_idtype.h"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_object.h"

#include "BLI_math_vector_types.hh"
#include "BLI_memarena.h"
#include "BLI_memory_utils.hh"
#include "BLI_polyfill_2d.h"
#include "BLI_span.hh"
#include "BLI_stack.hh"
#include "BLI_string.h"

#include "BLO_read_write.h"

#include "BLT_translation.h"

#include "DNA_ID.h"
#include "DNA_ID_enums.h"
#include "DNA_grease_pencil_types.h"
#include "DNA_material_types.h"

#include "MEM_guardedalloc.h"

using blender::float3;
using blender::Span;
using blender::uint3;
using blender::Vector;

static void grease_pencil_init_data(ID *id)
{
  using namespace blender::bke;

  GreasePencil *grease_pencil = (GreasePencil *)id;
  grease_pencil->runtime = MEM_new<GreasePencilRuntime>(__func__);
}

static void grease_pencil_copy_data(Main * /*bmain*/,
                                    ID *id_dst,
                                    const ID *id_src,
                                    const int /*flag*/)
{
  using namespace blender;

  GreasePencil *grease_pencil_dst = (GreasePencil *)id_dst;
  const GreasePencil *grease_pencil_src = (GreasePencil *)id_src;

  /* Duplicate material array. */
  grease_pencil_dst->material_array = static_cast<Material **>(
      MEM_dupallocN(grease_pencil_src->material_array));

  /* Duplicate drawing array. */
  grease_pencil_dst->drawing_array_size = grease_pencil_src->drawing_array_size;
  grease_pencil_dst->drawing_array = MEM_cnew_array<GreasePencilDrawingBase *>(
      grease_pencil_src->drawing_array_size, __func__);
  for (int i = 0; i < grease_pencil_src->drawing_array_size; i++) {
    const GreasePencilDrawingBase *src_drawing_base = grease_pencil_src->drawing_array[i];
    switch (src_drawing_base->type) {
      case GP_DRAWING: {
        const GreasePencilDrawing *src_drawing = reinterpret_cast<const GreasePencilDrawing *>(
            src_drawing_base);
        grease_pencil_dst->drawing_array[i] = reinterpret_cast<GreasePencilDrawingBase *>(
            MEM_cnew<GreasePencilDrawing>(__func__));
        GreasePencilDrawing *dst_drawing = reinterpret_cast<GreasePencilDrawing *>(
            grease_pencil_dst->drawing_array[i]);

        dst_drawing->base.type = src_drawing->base.type;
        dst_drawing->base.flag = src_drawing->base.flag;

        new (&dst_drawing->geometry) bke::CurvesGeometry(src_drawing->geometry.wrap());
        dst_drawing->runtime = MEM_new<bke::GreasePencilDrawingRuntime>(__func__);
        dst_drawing->runtime->triangles_cache = src_drawing->runtime->triangles_cache;
        break;
      }
      case GP_DRAWING_REFERENCE: {
        const GreasePencilDrawingReference *src_drawing_reference =
            reinterpret_cast<const GreasePencilDrawingReference *>(src_drawing_base);
        grease_pencil_dst->drawing_array[i] = reinterpret_cast<GreasePencilDrawingBase *>(
            MEM_dupallocN(src_drawing_reference));
        break;
      }
    }
  }

  /* Do not copy layer tree storage. */
  grease_pencil_dst->layer_tree_storage.nodes = nullptr;
  grease_pencil_dst->layer_tree_storage.nodes_num = 0;

  /* Duplicate runtime data. */
  if (grease_pencil_src->runtime) {
    grease_pencil_dst->runtime = MEM_new<bke::GreasePencilRuntime>(__func__,
                                                                   *grease_pencil_src->runtime);
  }
}

static void grease_pencil_free_data(ID *id)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;
  BKE_animdata_free(&grease_pencil->id, false);

  MEM_SAFE_FREE(grease_pencil->material_array);

  grease_pencil->free_drawing_array();
  grease_pencil->free_layer_tree_storage();

  BKE_grease_pencil_batch_cache_free(grease_pencil);

  MEM_delete(grease_pencil->runtime);
  grease_pencil->runtime = nullptr;
}

static void grease_pencil_foreach_id(ID *id, LibraryForeachIDData *data)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;
  for (int i = 0; i < grease_pencil->material_array_size; i++) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, grease_pencil->material_array[i], IDWALK_CB_USER);
  }
  for (int i = 0; i < grease_pencil->drawing_array_size; i++) {
    GreasePencilDrawingBase *drawing_base = grease_pencil->drawing_array[i];
    if (drawing_base->type == GP_DRAWING_REFERENCE) {
      GreasePencilDrawingReference *drawing_reference =
          reinterpret_cast<GreasePencilDrawingReference *>(drawing_base);
      BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, drawing_reference->id_reference, IDWALK_CB_USER);
    }
  }
}

static void grease_pencil_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;

  /* Free storage if there is one. */
  grease_pencil->free_layer_tree_storage();
  grease_pencil->save_layer_tree_to_storage();

  /* Write animation data. */
  if (grease_pencil->adt) {
    BKE_animdata_blend_write(writer, grease_pencil->adt);
  }

  /* Write LibData */
  BLO_write_id_struct(writer, GreasePencil, id_address, &grease_pencil->id);
  BKE_id_blend_write(writer, &grease_pencil->id);

  /* Write drawings. */
  grease_pencil->write_drawing_array(writer);
  /* Write layer tree. */
  grease_pencil->write_layer_tree_storage(writer);
  /* Free the layer tree storage again after writing. */
  grease_pencil->free_layer_tree_storage();
  /* Write materials. */
  BLO_write_pointer_array(
      writer, grease_pencil->material_array_size, grease_pencil->material_array);
}

static void grease_pencil_blend_read_data(BlendDataReader *reader, ID *id)
{
  using namespace blender::bke::greasepencil;
  GreasePencil *grease_pencil = (GreasePencil *)id;

  /* Read animation data. */
  BLO_read_data_address(reader, &grease_pencil->adt);
  BKE_animdata_blend_read_data(reader, grease_pencil->adt);

  /* Read drawings. */
  grease_pencil->read_drawing_array(reader);
  /* Read layer tree. */
  grease_pencil->read_layer_tree_storage(reader);
  /* Read materials. */
  BLO_read_pointer_array(reader, (void **)&grease_pencil->material_array);

  grease_pencil->runtime = MEM_new<blender::bke::GreasePencilRuntime>(__func__);
  grease_pencil->load_layer_tree_from_storage();
  /* Free the layer tree storage again after loading. */
  grease_pencil->free_layer_tree_storage();
}

static void grease_pencil_blend_read_lib(BlendLibReader *reader, ID *id)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;
  for (int i = 0; i < grease_pencil->material_array_size; i++) {
    BLO_read_id_address(reader, grease_pencil->id.lib, &grease_pencil->material_array[i]);
  }
  for (int i = 0; i < grease_pencil->drawing_array_size; i++) {
    GreasePencilDrawingBase *drawing_base = grease_pencil->drawing_array[i];
    if (drawing_base->type == GP_DRAWING_REFERENCE) {
      GreasePencilDrawingReference *drawing_reference =
          reinterpret_cast<GreasePencilDrawingReference *>(drawing_base);
      BLO_read_id_address(reader, grease_pencil->id.lib, &drawing_reference->id_reference);
    }
  }
}

static void grease_pencil_blend_read_expand(BlendExpander *expander, ID *id)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;
  for (int i = 0; i < grease_pencil->material_array_size; i++) {
    BLO_expand(expander, grease_pencil->material_array[i]);
  }
  for (int i = 0; i < grease_pencil->drawing_array_size; i++) {
    GreasePencilDrawingBase *drawing_base = grease_pencil->drawing_array[i];
    if (drawing_base->type == GP_DRAWING_REFERENCE) {
      GreasePencilDrawingReference *drawing_reference =
          reinterpret_cast<GreasePencilDrawingReference *>(drawing_base);
      BLO_expand(expander, drawing_reference->id_reference);
    }
  }
}

IDTypeInfo IDType_ID_GP = {
    /*id_code*/ ID_GP,
    /*id_filter*/ FILTER_ID_GP,
    /*main_listbase_index*/ INDEX_ID_GP,
    /*struct_size*/ sizeof(GreasePencil),
    /*name*/ "GreasePencil",
    /*name_plural*/ "grease_pencils_new",
    /*translation_context*/ BLT_I18NCONTEXT_ID_GPENCIL,
    /*flags*/ IDTYPE_FLAGS_APPEND_IS_REUSABLE,
    /*asset_type_info*/ nullptr,

    /*init_data*/ grease_pencil_init_data,
    /*copy_data*/ grease_pencil_copy_data,
    /*free_data*/ grease_pencil_free_data,
    /*make_local*/ nullptr,
    /*foreach_id*/ grease_pencil_foreach_id,
    /*foreach_cache*/ nullptr,
    /*foreach_path*/ nullptr,
    /*owner_pointer_get*/ nullptr,

    /*blend_write*/ grease_pencil_blend_write,
    /*blend_read_data*/ grease_pencil_blend_read_data,
    /*blend_read_lib*/ grease_pencil_blend_read_lib,
    /*blend_read_expand*/ grease_pencil_blend_read_expand,

    /*blend_read_undo_preserve*/ nullptr,

    /*lib_override_apply_post*/ nullptr,
};

namespace blender::bke::greasepencil {

TreeNode::TreeNode(GreasePencilLayerTreeNodeType type)
{
  this->type = type;
  this->name = nullptr;
}

TreeNode::TreeNode(GreasePencilLayerTreeNodeType type, StringRefNull name)
{
  this->type = type;
  this->name = BLI_strdup(name.c_str());
}

TreeNode::TreeNode(const TreeNode &other)
    : TreeNode::TreeNode(GreasePencilLayerTreeNodeType(other.type))
{
  if (other.name) {
    this->name = BLI_strdup(other.name);
  }
  this->flag = other.flag;
  copy_v3_v3_uchar(this->color, other.color);
}

TreeNode::~TreeNode()
{
  if (this->name) {
    MEM_freeN(this->name);
  }
}

const LayerGroup &TreeNode::as_group() const
{
  return *static_cast<const LayerGroup *>(this);
}

const Layer &TreeNode::as_layer() const
{
  return *static_cast<const Layer *>(this);
}

LayerGroup &TreeNode::as_group_for_write()
{
  return *static_cast<LayerGroup *>(this);
}

Layer &TreeNode::as_layer_for_write()
{
  return *static_cast<Layer *>(this);
}

LayerMask::LayerMask()
{
  this->layer_name = nullptr;
}

LayerMask::LayerMask(StringRefNull name)
{
  this->layer_name = BLI_strdup(name.c_str());
}

LayerMask::LayerMask(const LayerMask &other) : LayerMask()
{
  if (other.layer_name) {
    this->layer_name = BLI_strdup(other.layer_name);
  }
  this->flag = other.flag;
}

LayerMask::~LayerMask()
{
  if (this->layer_name) {
    MEM_freeN(this->layer_name);
  }
}

Layer::Layer() : TreeNode::TreeNode(GP_LAYER_TREE_LEAF), frames_(), masks_()
{
  this->parsubstr = nullptr;
  this->viewlayer_name = nullptr;
}

Layer::Layer(StringRefNull name)
    : TreeNode::TreeNode(GP_LAYER_TREE_LEAF, name), frames_(), masks_()
{
  this->parsubstr = nullptr;
  this->viewlayer_name = nullptr;
}

Layer::Layer(const Layer &other)
    : TreeNode::TreeNode(other), frames_(other.frames_), masks_(other.masks_)
{
  this->parent_type = other.parent_type;
  this->blend_mode = other.blend_mode;
  this->thickness_adjustment = other.thickness_adjustment;
  this->parent = other.parent;
  this->parsubstr = nullptr;
  this->viewlayer_name = nullptr;
  if (other.parsubstr != nullptr) {
    this->parsubstr = BLI_strdup(other.parsubstr);
  }
  if (other.viewlayer_name != nullptr) {
    this->viewlayer_name = BLI_strdup(other.viewlayer_name);
  }
  this->opacity = other.opacity;
  copy_v4_v4(this->tint_color, other.tint_color);
  copy_v3_v3(this->location, other.location);
  copy_v3_v3(this->rotation, other.rotation);
  copy_v3_v3(this->scale, other.scale);
}

Layer::~Layer()
{
  if (this->parsubstr != nullptr) {
    MEM_freeN(this->parsubstr);
  }
  if (this->viewlayer_name != nullptr) {
    MEM_freeN(this->viewlayer_name);
  }
}

const Map<int, GreasePencilFrame> &Layer::frames() const
{
  return this->frames_;
}

Map<int, GreasePencilFrame> &Layer::frames_for_write()
{
  return this->frames_;
}

const Vector<LayerMask> &Layer::masks() const
{
  return this->masks_;
}

Vector<LayerMask> &Layer::masks_for_write()
{
  return this->masks_;
}

bool Layer::is_visible() const
{
  return (this->flag & GP_LAYER_TREE_NODE_HIDE) == 0;
}

bool Layer::is_locked() const
{
  return (this->flag & GP_LAYER_TREE_NODE_LOCKED) != 0;
}

bool Layer::insert_frame(int frame_number, GreasePencilFrame &frame)
{
  return this->frames_for_write().add(frame_number, frame);
}

bool Layer::insert_frame(int frame_number, GreasePencilFrame &&frame)
{
  return this->frames_for_write().add(frame_number, frame);
}

bool Layer::overwrite_frame(int frame_number, GreasePencilFrame &frame)
{
  return this->frames_for_write().add_overwrite(frame_number, frame);
}

bool Layer::overwrite_frame(int frame_number, GreasePencilFrame &&frame)
{
  return this->frames_for_write().add_overwrite(frame_number, frame);
}

Span<int> Layer::sorted_keys() const
{
  this->sorted_keys_cache_.ensure([&](Vector<int> &r_data) {
    r_data.clear_and_shrink();
    r_data.reserve(this->frames().size());
    for (int64_t key : this->frames().keys()) {
      r_data.append(key);
    }
    std::sort(r_data.begin(), r_data.end());
  });
  return this->sorted_keys_cache_.data().as_span();
}

int Layer::drawing_index_at(int frame) const
{
  Span<int> sorted_keys = this->sorted_keys();
  /* No keyframes, return no drawing. */
  if (sorted_keys.size() == 0) {
    return -1;
  }
  /* Before the first drawing, return no drawing. */
  if (frame < sorted_keys.first()) {
    return -1;
  }
  /* After or at the last drawing, return the last drawing. */
  if (frame >= sorted_keys.last()) {
    return this->frames().lookup(sorted_keys.last()).drawing_index;
  }
  /* Search for the drawing. upper_bound will get the drawing just after. */
  auto it = std::upper_bound(sorted_keys.begin(), sorted_keys.end(), frame);
  if (it == sorted_keys.end() || it == sorted_keys.begin()) {
    return -1;
  }
  return this->frames().lookup(*std::prev(it)).drawing_index;
}

void Layer::tag_frames_map_keys_changed()
{
  this->sorted_keys_cache_.tag_dirty();
}

LayerGroup::LayerGroup(const LayerGroup &other) : TreeNode(other)
{
  this->children.reserve(other.children.size());
  for (const std::unique_ptr<TreeNode> &elem : other.children) {
    if (elem.get()->is_group()) {
      this->children.append(std::make_unique<LayerGroup>(elem.get()->as_group()));
    }
    else if (elem.get()->is_layer()) {
      this->children.append(std::make_unique<Layer>(elem.get()->as_layer()));
    }
  }
}

void LayerGroup::add_group(LayerGroup &group)
{
  this->children.append(std::make_unique<LayerGroup>(group));
}

void LayerGroup::add_group(LayerGroup &&group)
{
  this->children.append(std::make_unique<LayerGroup>(group));
}

Layer &LayerGroup::add_layer(Layer &layer)
{
  int64_t index = children.append_and_get_index(std::make_unique<Layer>(layer));
  return children[index].get()->as_layer_for_write();
}

Layer &LayerGroup::add_layer(Layer &&layer)
{
  int64_t index = children.append_and_get_index(std::make_unique<Layer>(layer));
  return children[index].get()->as_layer_for_write();
}

int64_t LayerGroup::num_direct_children() const
{
  return children.size();
}

int64_t LayerGroup::num_children_total() const
{
  int64_t total = 0;
  Stack<TreeNode *> stack;
  for (auto it = this->children.rbegin(); it != this->children.rend(); it++) {
    stack.push((*it).get());
  }
  while (!stack.is_empty()) {
    TreeNode &next_node = *stack.pop();
    total++;
    for (auto it = next_node.children.rbegin(); it != next_node.children.rend(); it++) {
      stack.push((*it).get());
    }
  }
  return total;
}

void LayerGroup::remove_child(int64_t index)
{
  BLI_assert(index >= 0 && index < this->children.size());
  this->children.remove(index);
}

void LayerGroup::foreach_children_pre_order(TreeNodeIterFn function)
{
  for (auto &child : this->children) {
    function(*child);
    if (child->is_group()) {
      child->as_group_for_write().foreach_children_pre_order(function);
    }
  }
}

void LayerGroup::foreach_children_with_index_pre_order(TreeNodeIndexIterFn function)
{
  Vector<TreeNode *> children = this->children_in_pre_order();
  for (const int64_t i : IndexRange(children.size())) {
    function(i, *children[i]);
  }
}

Vector<TreeNode *> LayerGroup::children_in_pre_order() const
{
  Vector<TreeNode *> children;

  Stack<TreeNode *> stack;
  for (auto it = this->children.rbegin(); it != this->children.rend(); it++) {
    stack.push((*it).get());
  }
  while (!stack.is_empty()) {
    TreeNode &next_node = *stack.pop();
    children.append(&next_node);
    for (auto it = next_node.children.rbegin(); it != next_node.children.rend(); it++) {
      stack.push((*it).get());
    }
  }
  return children;
}

Vector<Layer *> LayerGroup::layers_in_pre_order() const
{
  Vector<Layer *> layers;

  Stack<TreeNode *> stack;
  for (auto it = this->children.rbegin(); it != this->children.rend(); it++) {
    stack.push((*it).get());
  }
  while (!stack.is_empty()) {
    TreeNode &next_node = *stack.pop();
    if (next_node.is_layer()) {
      layers.append(&next_node.as_layer_for_write());
    }
    else {
      for (auto it = next_node.children.rbegin(); it != next_node.children.rend(); it++) {
        stack.push((*it).get());
      }
    }
  }
  return layers;
}

}  // namespace blender::bke::greasepencil

/* ------------------------------------------------------------------- */
/** \name GreasePencilRuntime functions
 * \{ */

namespace blender::bke {

GreasePencilRuntime::GreasePencilRuntime(const GreasePencilRuntime &other)
    : root_group_(other.root_group_), active_layer_index_(other.active_layer_index_)
{
  layer_cache_ = other.layer_cache_;
}

const greasepencil::LayerGroup &GreasePencilRuntime::root_group() const
{
  return this->root_group_;
}

greasepencil::LayerGroup &GreasePencilRuntime::root_group_for_write()
{
  return this->root_group_;
}

bool GreasePencilRuntime::has_active_layer() const
{
  return this->active_layer_index_ >= 0;
}

const greasepencil::Layer &GreasePencilRuntime::active_layer() const
{
  BLI_assert(this->active_layer_index_ >= 0);
  return *get_active_layer_from_index(this->active_layer_index_);
}

greasepencil::Layer &GreasePencilRuntime::active_layer_for_write() const
{
  BLI_assert(this->active_layer_index_ >= 0);
  return *get_active_layer_from_index(this->active_layer_index_);
}

void GreasePencilRuntime::set_active_layer_index(int index)
{
  this->active_layer_index_ = index;
}

int GreasePencilRuntime::active_layer_index() const
{
  return this->active_layer_index_;
}

void GreasePencilRuntime::ensure_layer_cache() const
{
  this->layer_cache_.ensure([this](Vector<greasepencil::Layer *> &data) {
    data = this->root_group_.layers_in_pre_order();
  });
}

static void read_layer_from_storage(blender::bke::greasepencil::LayerGroup &current_group,
                                    GreasePencilLayerTreeLeaf *node_leaf)
{
  using namespace blender::bke::greasepencil;
  Layer new_layer(node_leaf->base.name);
  for (int i = 0; i < node_leaf->layer.frames_storage.size; i++) {
    new_layer.insert_frame(node_leaf->layer.frames_storage.keys[i],
                           node_leaf->layer.frames_storage.values[i]);
  }
  new_layer.parent_type = node_leaf->layer.parent_type;
  new_layer.blend_mode = node_leaf->layer.blend_mode;
  new_layer.parent = node_leaf->layer.parent;
  if (node_leaf->layer.parsubstr) {
    new_layer.parsubstr = BLI_strdup(node_leaf->layer.parsubstr);
  }
  if (node_leaf->layer.viewlayer_name) {
    new_layer.viewlayer_name = BLI_strdup(node_leaf->layer.viewlayer_name);
  }
  Vector<LayerMask> masks = new_layer.masks_for_write();
  LISTBASE_FOREACH (bGPDlayer_Mask *, mask, &node_leaf->layer.masks_storage) {
    LayerMask new_mask = LayerMask(mask->name);
    new_mask.flag = mask->flag;
    masks.append(std::move(new_mask));
  }
  new_layer.opacity = node_leaf->layer.opacity;
  copy_v4_v4(new_layer.tint_color, node_leaf->layer.tint_color);
  copy_v3_v3(new_layer.location, node_leaf->layer.location);
  copy_v3_v3(new_layer.rotation, node_leaf->layer.rotation);
  copy_v3_v3(new_layer.scale, node_leaf->layer.scale);
  current_group.add_layer(std::move(new_layer));
}

static int read_layer_node_recursive(blender::bke::greasepencil::LayerGroup &current_group,
                                     GreasePencilLayerTreeNode **nodes,
                                     int node_index)
{
  using namespace blender::bke::greasepencil;
  GreasePencilLayerTreeNode *node = nodes[node_index];
  int total_nodes_read = 0;
  switch (node->type) {
    case GP_LAYER_TREE_LEAF: {
      GreasePencilLayerTreeLeaf *node_leaf = reinterpret_cast<GreasePencilLayerTreeLeaf *>(node);
      read_layer_from_storage(current_group, node_leaf);
      total_nodes_read++;
      break;
    }
    case GP_LAYER_TREE_GROUP: {
      GreasePencilLayerTreeGroup *group = reinterpret_cast<GreasePencilLayerTreeGroup *>(node);
      /* Read layer group. */
      LayerGroup new_group(group->base.name);
      for (int i = 0; i < group->children_num; i++) {
        total_nodes_read += read_layer_node_recursive(
            new_group, nodes, node_index + total_nodes_read + 1);
      }
      current_group.add_group(std::move(new_group));
      total_nodes_read++;
      break;
    }
  }
  return total_nodes_read;
}

void GreasePencilRuntime::load_layer_tree_from_storage(GreasePencilLayerTreeStorage &storage)
{
  using namespace blender::bke::greasepencil;
  if (storage.nodes_num == 0 || !storage.nodes) {
    return;
  }
  /* The first node should be the root group. */
  GreasePencilLayerTreeNode *root = reinterpret_cast<GreasePencilLayerTreeNode *>(
      storage.nodes[0]);
  BLI_assert(root->type == GP_LAYER_TREE_GROUP);
  int total_nodes_read = 0;
  for (int i = 0; i < reinterpret_cast<GreasePencilLayerTreeGroup *>(root)->children_num; i++) {
    total_nodes_read += read_layer_node_recursive(
        this->root_group_, storage.nodes, total_nodes_read + 1);
  }
  BLI_assert(total_nodes_read + 1 == storage.nodes_num);
  this->set_active_layer_index(storage.active_layer_index);

  this->tag_layer_tree_topology_changed();
}

static void save_tree_node_to_storage(const blender::bke::greasepencil::TreeNode &node,
                                      GreasePencilLayerTreeNode *dst)
{
  dst->type = node.type;
  copy_v3_v3_uchar(dst->color, node.color);
  dst->flag = node.flag;
  if (node.name) {
    dst->name = BLI_strdup(node.name);
  }
}

static void save_layer_to_storage(const blender::bke::greasepencil::Layer &node,
                                  GreasePencilLayerTreeNode **dst)
{
  using namespace blender;
  GreasePencilLayerTreeLeaf *new_leaf = MEM_cnew<GreasePencilLayerTreeLeaf>(__func__);
  /* Save properties. */
  save_tree_node_to_storage(node, &new_leaf->base);

  /* Save frames map. */
  const int frames_size = node.frames().size();
  if (frames_size > 0) {
    new_leaf->layer.frames_storage.size = frames_size;
    new_leaf->layer.frames_storage.keys = MEM_cnew_array<int>(frames_size, __func__);
    new_leaf->layer.frames_storage.values = MEM_cnew_array<GreasePencilFrame>(frames_size,
                                                                              __func__);

    MutableSpan<int> keys{new_leaf->layer.frames_storage.keys, frames_size};
    MutableSpan<GreasePencilFrame> values{new_leaf->layer.frames_storage.values, frames_size};
    keys.copy_from(node.sorted_keys());
    for (int i : keys.index_range()) {
      values[i] = node.frames().lookup(keys[i]);
    }
  }

  new_leaf->layer.parent_type = node.parent_type;
  new_leaf->layer.blend_mode = node.blend_mode;
  new_leaf->layer.parent = node.parent;
  if (node.parsubstr) {
    new_leaf->layer.parsubstr = BLI_strdup(node.parsubstr);
  }
  if (node.viewlayer_name) {
    new_leaf->layer.viewlayer_name = BLI_strdup(node.viewlayer_name);
  }
  BLI_listbase_clear(&new_leaf->layer.masks_storage);
  for (const bke::greasepencil::LayerMask &mask : node.masks()) {
    GreasePencilLayerMask *new_mask = MEM_cnew<GreasePencilLayerMask>(__func__);
    new_mask->layer_name = BLI_strdup(mask.layer_name);
    new_mask->flag = mask.flag;
    BLI_addtail(&new_leaf->layer.masks_storage, new_mask);
  }
  new_leaf->layer.opacity = node.opacity;
  copy_v4_v4(new_leaf->layer.tint_color, node.tint_color);
  copy_v3_v3(new_leaf->layer.location, node.location);
  copy_v3_v3(new_leaf->layer.rotation, node.rotation);
  copy_v3_v3(new_leaf->layer.scale, node.scale);

  /* Store pointer. */
  *dst = reinterpret_cast<GreasePencilLayerTreeNode *>(new_leaf);
}

static void save_layer_group_to_storage(const blender::bke::greasepencil::LayerGroup &node,
                                        GreasePencilLayerTreeNode **dst)
{
  GreasePencilLayerTreeGroup *new_group = MEM_cnew<GreasePencilLayerTreeGroup>(__func__);
  /* Save properties. */
  save_tree_node_to_storage(node, &new_group->base);

  /* Save number of children. */
  new_group->children_num = node.num_direct_children();

  /* Store pointer. */
  *dst = reinterpret_cast<GreasePencilLayerTreeNode *>(new_group);
}

void GreasePencilRuntime::save_layer_tree_to_storage(GreasePencilLayerTreeStorage &storage)
{
  using namespace blender::bke::greasepencil;
  /* We always store the root group, so we have to add one here. */
  int num_tree_nodes = this->root_group_.num_children_total() + 1;
  storage.nodes_num = num_tree_nodes;
  storage.nodes = MEM_cnew_array<GreasePencilLayerTreeNode *>(num_tree_nodes, __func__);

  save_layer_group_to_storage(this->root_group_, &storage.nodes[0]);
  this->root_group_.foreach_children_with_index_pre_order([&](uint64_t i, TreeNode &node) {
    GreasePencilLayerTreeNode **dst = &storage.nodes[i + 1];
    if (node.is_group()) {
      const LayerGroup &group = node.as_group();
      save_layer_group_to_storage(group, dst);
    }
    else if (node.is_layer()) {
      const Layer &layer = node.as_layer();
      save_layer_to_storage(layer, dst);
    }
  });

  storage.active_layer_index = this->active_layer_index_;
}

void GreasePencilRuntime::tag_layer_tree_topology_changed()
{
  this->layer_cache_.tag_dirty();
}

greasepencil::Layer *GreasePencilRuntime::get_active_layer_from_index(int index) const
{
  this->ensure_layer_cache();
  return this->layer_cache_.data()[index];
}

}  // namespace blender::bke

/** \} */

/* ------------------------------------------------------------------- */
/** \name Grease Pencil kernel functions
 * \{ */

void *BKE_grease_pencil_add(Main *bmain, const char *name)
{
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(BKE_id_new(bmain, ID_GP, name));

  return grease_pencil;
}

GreasePencil *BKE_grease_pencil_new_nomain()
{
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(BKE_id_new_nomain(ID_GP, nullptr));
  grease_pencil_init_data(&grease_pencil->id);
  return grease_pencil;
}

BoundBox *BKE_grease_pencil_boundbox_get(Object *ob)
{
  BLI_assert(ob->type == OB_GREASE_PENCIL);
  const GreasePencil *grease_pencil = static_cast<const GreasePencil *>(ob->data);

  if (ob->runtime.bb != nullptr && (ob->runtime.bb->flag & BOUNDBOX_DIRTY) == 0) {
    return ob->runtime.bb;
  }

  if (ob->runtime.bb == nullptr) {
    ob->runtime.bb = MEM_cnew<BoundBox>(__func__);

    float3 min(FLT_MAX);
    float3 max(-FLT_MAX);

    /* FIXME: this should somehow go through the visible drawings. We don't have access to the
     * scene time here, so we probably need to cache the visible drawing for each layer somehow. */
    for (int i = 0; i < grease_pencil->drawing_array_size; i++) {
      GreasePencilDrawingBase *drawing_base = grease_pencil->drawing_array[i];
      switch (drawing_base->type) {
        case GP_DRAWING: {
          GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);
          const blender::bke::CurvesGeometry &curves = drawing->geometry.wrap();

          if (!curves.bounds_min_max(min, max)) {
            blender::math::min_max(float3(-1), min, max);
            blender::math::min_max(float3(1), min, max);
          }
          break;
        }
        case GP_DRAWING_REFERENCE: {
          /* TODO */
          break;
        }
      }
    }

    BKE_boundbox_init_from_minmax(ob->runtime.bb, min, max);
  }

  return ob->runtime.bb;
}

void BKE_grease_pencil_data_update(struct Depsgraph * /*depsgraph*/,
                                   struct Scene * /*scene*/,
                                   Object *object)
{
  /* Free any evaluated data and restore original data. */
  BKE_object_free_derived_caches(object);

  GreasePencil *grease_pencil = static_cast<GreasePencil *>(object->data);
  /* Evaluate modifiers. */
  /* TODO. */

  /* Assign evaluated object. */
  /* TODO: Get eval from modifiers geometry set. */
  GreasePencil *grease_pencil_eval = (GreasePencil *)BKE_id_copy_ex(
      nullptr, &grease_pencil->id, nullptr, LIB_ID_COPY_LOCALIZE);
  // if (grease_pencil_eval == nullptr) {
  //   grease_pencil_eval = BKE_grease_pencil_new_nomain();
  //   BKE_object_eval_assign_data(object, &grease_pencil_eval->id, true);
  // }
  BKE_object_eval_assign_data(object, &grease_pencil_eval->id, true);
}

/** \} */

/* ------------------------------------------------------------------- */
/** \name Draw Cache
 * \{ */

void (*BKE_grease_pencil_batch_cache_dirty_tag_cb)(GreasePencil *grease_pencil,
                                                   int mode) = nullptr;
void (*BKE_grease_pencil_batch_cache_free_cb)(GreasePencil *grease_pencil) = nullptr;

void BKE_grease_pencil_batch_cache_dirty_tag(GreasePencil *grease_pencil, int mode)
{
  if (grease_pencil->runtime && grease_pencil->runtime->batch_cache) {
    BKE_grease_pencil_batch_cache_dirty_tag_cb(grease_pencil, mode);
  }
}

void BKE_grease_pencil_batch_cache_free(GreasePencil *grease_pencil)
{
  if (grease_pencil->runtime && grease_pencil->runtime->batch_cache) {
    BKE_grease_pencil_batch_cache_free_cb(grease_pencil);
  }
}

/** \} */

/* ------------------------------------------------------------------- */
/** \name Grease Pencil Drawing API
 * \{ */

blender::Span<blender::uint3> GreasePencilDrawing::triangles() const
{
  using namespace blender;
  const bke::GreasePencilDrawingRuntime &runtime = *this->runtime;
  runtime.triangles_cache.ensure([&](Vector<uint3> &cache) {
    MemArena *pf_arena = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, __func__);

    const bke::CurvesGeometry &curves = this->geometry.wrap();
    const Span<float3> positions = curves.positions();
    const offset_indices::OffsetIndices<int> points_by_curve = curves.points_by_curve();

    int total_triangles = 0;
    for (int curve_i : curves.curves_range()) {
      IndexRange points = points_by_curve[curve_i];
      if (points.size() > 2) {
        total_triangles += points.size() - 2;
      }
    }

    cache.resize(total_triangles);

    int t = 0;
    for (int curve_i : curves.curves_range()) {
      const IndexRange points = points_by_curve[curve_i];

      if (points.size() < 3) {
        continue;
      }

      const int num_trinagles = points.size() - 2;

      uint(*tris)[3] = static_cast<uint(*)[3]>(
          BLI_memarena_alloc(pf_arena, sizeof(*tris) * size_t(num_trinagles)));
      float(*projverts)[2] = static_cast<float(*)[2]>(
          BLI_memarena_alloc(pf_arena, sizeof(*projverts) * size_t(points.size())));

      /* TODO: calculate axis_mat properly. */
      float3x3 axis_mat;
      axis_dominant_v3_to_m3(axis_mat.ptr(), float3(0.0f, -1.0f, 0.0f));

      for (const int i : IndexRange(points.size())) {
        mul_v2_m3v3(projverts[i], axis_mat.ptr(), positions[points[i]]);
      }

      BLI_polyfill_calc_arena(projverts, points.size(), 0, tris, pf_arena);

      for (const int i : IndexRange(num_trinagles)) {
        cache[t] = uint3(tris[i]);
        t++;
      }

      BLI_memarena_clear(pf_arena);
    }

    BLI_memarena_free(pf_arena);
  });

  return this->runtime->triangles_cache.data().as_span();
}

void GreasePencilDrawing::tag_positions_changed()
{
  this->geometry.wrap().tag_positions_changed();
  this->runtime->triangles_cache.tag_dirty();
}

bool GreasePencilDrawing::has_stroke_buffer()
{
  return this->runtime->stroke_cache.points.size() > 0;
}

blender::Span<blender::bke::StrokePoint> GreasePencilDrawing::stroke_buffer()
{
  return this->runtime->stroke_cache.points.as_span();
}

/** \} */

/* ------------------------------------------------------------------- */
/** \name Grease Pencil data-block API
 * \{ */

static void grease_pencil_grow_drawing_array_by(GreasePencil &self, const int add_capacity)
{
  BLI_assert(add_capacity > 0);
  const int new_drawing_array_size = self.drawing_array_size + add_capacity;
  GreasePencilDrawingBase **new_drawing_array = reinterpret_cast<GreasePencilDrawingBase **>(
      MEM_cnew_array<GreasePencilDrawingBase *>(new_drawing_array_size, __func__));

  blender::uninitialized_relocate_n(
      self.drawing_array, self.drawing_array_size, new_drawing_array);

  self.drawing_array = new_drawing_array;
  self.drawing_array_size = new_drawing_array_size;
}

static void grease_pencil_shrink_drawing_array_by(GreasePencil &self, const int remove_capacity)
{
  BLI_assert(remove_capacity > 0);
  const int new_drawing_array_size = self.drawing_array_size - remove_capacity;
  GreasePencilDrawingBase **new_drawing_array = reinterpret_cast<GreasePencilDrawingBase **>(
      MEM_cnew_array<GreasePencilDrawingBase *>(new_drawing_array_size, __func__));

  blender::uninitialized_move_n(self.drawing_array, new_drawing_array_size, new_drawing_array);
  MEM_freeN(self.drawing_array);

  self.drawing_array = new_drawing_array;
  self.drawing_array_size = new_drawing_array_size;
}

blender::Span<GreasePencilDrawingBase *> GreasePencil::drawings() const
{
  return blender::Span<GreasePencilDrawingBase *>{this->drawing_array, this->drawing_array_size};
}

blender::MutableSpan<GreasePencilDrawingBase *> GreasePencil::drawings_for_write()
{
  return blender::MutableSpan<GreasePencilDrawingBase *>{this->drawing_array,
                                                         this->drawing_array_size};
}

void GreasePencil::add_empty_drawings(int n)
{
  using namespace blender;
  BLI_assert(n > 0);
  const int prev_size = this->drawings().size();
  grease_pencil_grow_drawing_array_by(*this, n);
  MutableSpan<GreasePencilDrawingBase *> new_drawings = this->drawings_for_write().drop_front(
      prev_size);
  for (const int i : IndexRange(new_drawings.size())) {
    new_drawings[i] = reinterpret_cast<GreasePencilDrawingBase *>(
        MEM_new<GreasePencilDrawing>(__func__));
    GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(new_drawings[i]);
    new (&drawing->geometry) bke::CurvesGeometry();
    drawing->runtime = MEM_new<bke::GreasePencilDrawingRuntime>(__func__);
  }
}

void GreasePencil::remove_drawing(int index_to_remove)
{
  using namespace blender::bke::greasepencil;
  /* In order to not change the indices of the drawings, we do the following to the drawing to be
   * removed:
   *  - If the drawing (A) is not the last one:
   *     1.1) Find any frames in the layers that reference the last drawing (B) and point them to
   *          A's index.
   *     1.2) Swap drawing A with drawing B.
   *  2) Destroy A and shrink the array by one.
   *  3) Remove any frames in the layers that reference the A's index.
   */
  BLI_assert(this->drawing_array_size > 0);
  BLI_assert(index_to_remove >= 0 && index_to_remove < this->drawing_array_size);

  /* Move the drawing that should be removed to the last index. */
  const int last_drawing_index = this->drawing_array_size - 1;
  if (index_to_remove != last_drawing_index) {
    for (Layer *layer : this->layers_for_write()) {
      blender::Map<int, GreasePencilFrame> &frames = layer->frames_for_write();
      for (auto [key, value] : frames.items()) {
        if (value.drawing_index == last_drawing_index) {
          value.drawing_index = index_to_remove;
        }
        else if (value.drawing_index == index_to_remove) {
          value.drawing_index = last_drawing_index;
        }
      }
    }
    std::swap(this->drawings_for_write()[index_to_remove],
              this->drawings_for_write()[last_drawing_index]);
  }

  /* Delete the last drawing. */
  GreasePencilDrawingBase *drawing_base_to_remove = this->drawings_for_write()[last_drawing_index];
  switch (drawing_base_to_remove->type) {
    case GP_DRAWING: {
      GreasePencilDrawing *drawing_to_remove = reinterpret_cast<GreasePencilDrawing *>(
          drawing_base_to_remove);
      drawing_to_remove->geometry.wrap().~CurvesGeometry();
      MEM_delete(drawing_to_remove->runtime);
      drawing_to_remove->runtime = nullptr;
      MEM_freeN(drawing_to_remove);
      break;
    }
    case GP_DRAWING_REFERENCE: {
      GreasePencilDrawingReference *drawing_reference_to_remove =
          reinterpret_cast<GreasePencilDrawingReference *>(drawing_base_to_remove);
      MEM_freeN(drawing_reference_to_remove);
      break;
    }
  }

  /* Remove any frame that points to the last drawing. */
  for (Layer *layer : this->layers_for_write()) {
    blender::Map<int, GreasePencilFrame> &frames = layer->frames_for_write();
    int64_t frames_removed = frames.remove_if([last_drawing_index](auto item) {
      return item.value.drawing_index == last_drawing_index;
    });
    if (frames_removed > 0) {
      layer->tag_frames_map_keys_changed();
    }
  }

  /* Shrink drawing array. */
  grease_pencil_shrink_drawing_array_by(*this, 1);
}

void GreasePencil::foreach_visible_drawing(
    int frame, blender::FunctionRef<void(GreasePencilDrawing &)> function)
{
  using namespace blender::bke::greasepencil;

  blender::Span<GreasePencilDrawingBase *> drawings = this->drawings();
  for (const Layer *layer : this->layers()) {
    if (!layer->is_visible()) {
      continue;
    }
    int index = layer->drawing_index_at(frame);
    if (index == -1) {
      continue;
    }
    GreasePencilDrawingBase *drawing_baseerence = drawings[index];
    if (drawing_baseerence->type == GP_DRAWING) {
      GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_baseerence);
      function(*drawing);
    }
    else if (drawing_baseerence->type == GP_DRAWING_REFERENCE) {
      /* TODO */
    }
  }
}

const blender::bke::greasepencil::LayerGroup &GreasePencil::root_group() const
{
  BLI_assert(this->runtime != nullptr);
  return this->runtime->root_group();
}

blender::bke::greasepencil::LayerGroup &GreasePencil::root_group_for_write()
{
  BLI_assert(this->runtime != nullptr);
  return this->runtime->root_group_for_write();
}

blender::Span<const blender::bke::greasepencil::Layer *> GreasePencil::layers() const
{
  this->runtime->ensure_layer_cache();
  return this->runtime->layer_cache_.data();
}

blender::Span<blender::bke::greasepencil::Layer *> GreasePencil::layers_for_write()
{
  this->runtime->ensure_layer_cache();
  return this->runtime->layer_cache_.data();
}

void GreasePencil::tag_layer_tree_topology_changed()
{
  this->runtime->tag_layer_tree_topology_changed();
}

/** \} */

/* ------------------------------------------------------------------- */
/** \name Drawing array storage functions
 * \{ */

void GreasePencil::read_drawing_array(BlendDataReader *reader)
{
  BLO_read_pointer_array(reader, (void **)&this->drawing_array);
  for (int i = 0; i < this->drawing_array_size; i++) {
    BLO_read_data_address(reader, &this->drawing_array[i]);
    GreasePencilDrawingBase *drawing_base = this->drawing_array[i];
    switch (drawing_base->type) {
      case GP_DRAWING: {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);
        drawing->geometry.wrap().blend_read(*reader);
        /* Initialize runtime data. */
        drawing->geometry.runtime = MEM_new<blender::bke::CurvesGeometryRuntime>(__func__);
        drawing->geometry.wrap().update_curve_types();
        drawing->runtime = MEM_new<blender::bke::GreasePencilDrawingRuntime>(__func__);
        drawing->runtime->triangles_cache.tag_dirty();
        break;
      }
      case GP_DRAWING_REFERENCE: {
        GreasePencilDrawingReference *drawing_reference =
            reinterpret_cast<GreasePencilDrawingReference *>(drawing_base);
        BLO_read_data_address(reader, &drawing_reference->id_reference);
        break;
      }
    }
  }
}

void GreasePencil::write_drawing_array(BlendWriter *writer)
{
  BLO_write_pointer_array(writer, this->drawing_array_size, this->drawing_array);
  for (int i = 0; i < this->drawing_array_size; i++) {
    GreasePencilDrawingBase *drawing_base = this->drawing_array[i];
    switch (drawing_base->type) {
      case GP_DRAWING: {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);
        BLO_write_struct(writer, GreasePencilDrawing, drawing);
        drawing->geometry.wrap().blend_write(*writer, this->id);
        break;
      }
      case GP_DRAWING_REFERENCE: {
        GreasePencilDrawingReference *drawing_reference =
            reinterpret_cast<GreasePencilDrawingReference *>(drawing_base);
        BLO_write_struct(writer, GreasePencilDrawingReference, drawing_reference);
        break;
      }
    }
  }
}

void GreasePencil::free_drawing_array()
{
  if (this->drawing_array == nullptr || this->drawing_array_size == 0) {
    return;
  }
  for (int i = 0; i < this->drawing_array_size; i++) {
    GreasePencilDrawingBase *drawing_base = this->drawing_array[i];
    switch (drawing_base->type) {
      case GP_DRAWING: {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);
        drawing->geometry.wrap().~CurvesGeometry();
        MEM_delete(drawing->runtime);
        drawing->runtime = nullptr;
        MEM_freeN(drawing);
        break;
      }
      case GP_DRAWING_REFERENCE: {
        GreasePencilDrawingReference *drawing_reference =
            reinterpret_cast<GreasePencilDrawingReference *>(drawing_base);
        MEM_freeN(drawing_reference);
        break;
      }
    }
  }
  MEM_freeN(this->drawing_array);
  this->drawing_array = nullptr;
  this->drawing_array_size = 0;
}

/** \} */

/* ------------------------------------------------------------------- */
/** \name Layer Tree storage functions
 * \{ */

void GreasePencil::save_layer_tree_to_storage()
{
  this->runtime->save_layer_tree_to_storage(this->layer_tree_storage);
}

void GreasePencil::load_layer_tree_from_storage()
{
  this->runtime->load_layer_tree_from_storage(this->layer_tree_storage);
}

void GreasePencil::read_layer_tree_storage(BlendDataReader *reader)
{
  BLO_read_pointer_array(reader, (void **)&this->layer_tree_storage.nodes);
  for (int i = 0; i < this->layer_tree_storage.nodes_num; i++) {
    BLO_read_data_address(reader, &this->layer_tree_storage.nodes[i]);
    GreasePencilLayerTreeNode *node = this->layer_tree_storage.nodes[i];
    switch (node->type) {
      case GP_LAYER_TREE_LEAF: {
        GreasePencilLayerTreeLeaf *node_leaf = reinterpret_cast<GreasePencilLayerTreeLeaf *>(node);
        BLO_read_data_address(reader, &node_leaf->base.name);
        /* Read layer data. */
        BLO_read_int32_array(
            reader, node_leaf->layer.frames_storage.size, &node_leaf->layer.frames_storage.keys);
        BLO_read_data_address(reader, &node_leaf->layer.frames_storage.values);
        BLO_read_data_address(reader, &node_leaf->layer.parsubstr);
        BLO_read_data_address(reader, &node_leaf->layer.viewlayer_name);
        BLO_read_list(reader, &node_leaf->layer.masks_storage);
        LISTBASE_FOREACH (GreasePencilLayerMask *, mask, &node_leaf->layer.masks_storage) {
          BLO_read_data_address(reader, &mask->layer_name);
        }
        break;
      }
      case GP_LAYER_TREE_GROUP: {
        GreasePencilLayerTreeGroup *group = reinterpret_cast<GreasePencilLayerTreeGroup *>(node);
        BLO_read_data_address(reader, &group->base.name);
        break;
      }
    }
  }
}

void GreasePencil::write_layer_tree_storage(BlendWriter *writer)
{
  BLO_write_pointer_array(
      writer, this->layer_tree_storage.nodes_num, this->layer_tree_storage.nodes);
  for (int i = 0; i < this->layer_tree_storage.nodes_num; i++) {
    GreasePencilLayerTreeNode *node = this->layer_tree_storage.nodes[i];
    switch (node->type) {
      case GP_LAYER_TREE_LEAF: {
        GreasePencilLayerTreeLeaf *node_leaf = reinterpret_cast<GreasePencilLayerTreeLeaf *>(node);
        BLO_write_struct(writer, GreasePencilLayerTreeLeaf, node_leaf);
        BLO_write_string(writer, node_leaf->base.name);
        /* Write layer data. */
        BLO_write_int32_array(
            writer, node_leaf->layer.frames_storage.size, node_leaf->layer.frames_storage.keys);
        BLO_write_struct_array(writer,
                               GreasePencilFrame,
                               node_leaf->layer.frames_storage.size,
                               node_leaf->layer.frames_storage.values);
        BLO_write_string(writer, node_leaf->layer.parsubstr);
        BLO_write_string(writer, node_leaf->layer.viewlayer_name);
        BLO_write_struct_list(writer, GreasePencilLayerMask, &node_leaf->layer.masks_storage);
        LISTBASE_FOREACH (GreasePencilLayerMask *, mask, &node_leaf->layer.masks_storage) {
          BLO_write_string(writer, mask->layer_name);
        }
        break;
      }
      case GP_LAYER_TREE_GROUP: {
        GreasePencilLayerTreeGroup *group = reinterpret_cast<GreasePencilLayerTreeGroup *>(node);
        BLO_write_struct(writer, GreasePencilLayerTreeGroup, group);
        BLO_write_string(writer, group->base.name);
        break;
      }
    }
  }
}

void GreasePencil::free_layer_tree_storage()
{
  if (this->layer_tree_storage.nodes_num == 0 || this->layer_tree_storage.nodes == nullptr) {
    return;
  }
  for (int i = 0; i < this->layer_tree_storage.nodes_num; i++) {
    GreasePencilLayerTreeNode *node = this->layer_tree_storage.nodes[i];
    if (node->name) {
      MEM_freeN(node->name);
    }
    switch (node->type) {
      case GP_LAYER_TREE_LEAF: {
        GreasePencilLayerTreeLeaf *node_leaf = reinterpret_cast<GreasePencilLayerTreeLeaf *>(node);
        if (node_leaf->layer.frames_storage.size > 0) {
          MEM_freeN(node_leaf->layer.frames_storage.keys);
          MEM_freeN(node_leaf->layer.frames_storage.values);
        }
        if (node_leaf->layer.parsubstr) {
          MEM_freeN(node_leaf->layer.parsubstr);
        }
        if (node_leaf->layer.viewlayer_name) {
          MEM_freeN(node_leaf->layer.viewlayer_name);
        }
        LISTBASE_FOREACH_MUTABLE (GreasePencilLayerMask *, mask, &node_leaf->layer.masks_storage) {
          if (mask->layer_name) {
            MEM_freeN(mask->layer_name);
          }
          MEM_freeN(mask);
        }
        MEM_freeN(node_leaf);
        break;
      }
      case GP_LAYER_TREE_GROUP: {
        GreasePencilLayerTreeGroup *group = reinterpret_cast<GreasePencilLayerTreeGroup *>(node);
        MEM_freeN(group);
        break;
      }
    }
  }
  MEM_freeN(this->layer_tree_storage.nodes);
  this->layer_tree_storage.nodes = nullptr;
  this->layer_tree_storage.nodes_num = 0;
}

/** \} */
