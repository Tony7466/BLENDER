/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_node_types.h"

#include "BLI_string.h"
#include "BLI_string_utils.h"

blender::Span<NodeEnumItem> NodeEnumDefinition::items() const
{
  return {this->items_array, this->items_num};
}

blender::MutableSpan<NodeEnumItem> NodeEnumDefinition::items_for_write()
{
  return {this->items_array, this->items_num};
}

NodeEnumItem *NodeEnumDefinition::add_item(blender::StringRef name)
{
  const int insert_index = this->items_num;
  NodeEnumItem *old_items = this->items_array;

  this->items_array = MEM_cnew_array<NodeEnumItem>(this->items_num + 1, __func__);
  std::copy_n(old_items, insert_index, this->items_array);
  NodeEnumItem &new_item = this->items_array[insert_index];
  std::copy_n(old_items + insert_index + 1,
              this->items_num - insert_index,
              this->items_array + insert_index + 1);

  new_item.identifier = this->next_identifier++;
  this->set_item_name(new_item, name);

  this->items_num++;
  MEM_SAFE_FREE(old_items);

  this->flag |= NODE_ENUM_DEFINITION_CHANGED;
  return &new_item;
}

bool NodeEnumDefinition::remove_item(NodeEnumItem &item)
{
  if (!this->items().contains_ptr(&item)) {
    return false;
  }
  const int remove_index = &item - this->items().begin();
  NodeEnumItem *old_items = this->items_array;

  this->items_array = MEM_cnew_array<NodeEnumItem>(this->items_num - 1, __func__);
  std::copy_n(old_items, remove_index, this->items_array);
  std::copy_n(old_items + remove_index + 1,
              this->items_num - remove_index - 1,
              this->items_array + remove_index + 1);

  this->items_num--;
  MEM_SAFE_FREE(old_items);

  this->flag |= NODE_ENUM_DEFINITION_CHANGED;
  return true;
}

void NodeEnumDefinition::clear()
{
  if (this->items_num == 0) {
    return;
  }
  this->items_num = 0;
  MEM_SAFE_FREE(this->items_array);

  this->flag |= NODE_ENUM_DEFINITION_CHANGED;
}

bool NodeEnumDefinition::move_item(uint16_t from_index, uint16_t to_index)
{
  if (from_index < 0 || from_index >= this->items_num || to_index < 0 ||
      to_index >= this->items_num) {
    return false;
  }

  if (from_index < to_index) {
    const NodeEnumItem tmp = this->items_array[from_index];
    std::copy(this->items_array + from_index + 1,
              this->items_array + to_index,
              this->items_array + from_index);
    this->items_array[to_index] = tmp;
  }
  else if (from_index > to_index) {
    const NodeEnumItem tmp = this->items_array[from_index];
    std::copy_backward(this->items_array + to_index,
                       this->items_array + from_index - 1,
                       this->items_array + to_index + 1);
    this->items_array[to_index] = tmp;
  }

  this->flag |= NODE_ENUM_DEFINITION_CHANGED;
  return true;
}

const NodeEnumItem *NodeEnumDefinition::active_item() const
{
  if (blender::IndexRange(this->items_num).contains(this->active_index)) {
    return &this->items()[this->active_index];
  }
  return nullptr;
}

NodeEnumItem *NodeEnumDefinition::active_item()
{
  if (blender::IndexRange(this->items_num).contains(this->active_index)) {
    return &this->items_for_write()[this->active_index];
  }
  return nullptr;
}

void NodeEnumDefinition::active_item_set(NodeEnumItem *item) {
  this->active_index = this->items().contains_ptr(item) ? item - this->items_array : -1;
}

void NodeEnumDefinition::set_item_name(NodeEnumItem &item, blender::StringRef name)
{
  char unique_name[MAX_NAME + 4];
  STRNCPY(unique_name, name.data());

  struct Args {
    NodeEnumDefinition *enum_def;
    const NodeEnumItem *item;
  } args = {this, &item};

  const char *default_name = items().is_empty() ? "Name" : items().last().name;
  BLI_uniquename_cb(
      [](void *arg, const char *name) {
        const Args &args = *static_cast<Args *>(arg);
        for (const NodeEnumItem &item : args.enum_def->items()) {
          if (&item != args.item) {
            if (STREQ(item.name, name)) {
              return true;
            }
          }
        }
        return false;
      },
      &args,
      default_name,
      '.',
      unique_name,
      ARRAY_SIZE(unique_name));

  MEM_SAFE_FREE(item.name);
  item.name = BLI_strdup(unique_name);

  this->flag |= NODE_ENUM_DEFINITION_CHANGED;
}
