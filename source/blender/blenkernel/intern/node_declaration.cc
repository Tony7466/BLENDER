/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node_declaration.hh"

#include "BLI_string.h"

#include "DNA_node_types.h"

int bNodeTreeDeclaration::socket_index(bNodeSocketDeclaration &socket_decl) const
{
  return sockets().first_index_try(&socket_decl);
}

bNodeSocketDeclaration *bNodeTreeDeclaration::add_socket(blender::StringRef name,
                                                         const eNodeSocketDeclarationInOut in_out)
{
  bNodeSocketDeclaration *new_socket = MEM_cnew<bNodeSocketDeclaration>(__func__);
  new_socket->name = BLI_strdup(name.data());
  new_socket->in_out = in_out;

  blender::MutableSpan<bNodeSocketDeclaration *> old_sockets = sockets();
  sockets_num++;
  sockets_array = MEM_cnew_array<bNodeSocketDeclaration *>(sockets_num, __func__);
  sockets().drop_back(1).copy_from(old_sockets);
  sockets().last() = new_socket;

  if (!old_sockets.is_empty()) {
    MEM_freeN(old_sockets.data());
  }

  return new_socket;
}

bNodeSocketDeclaration *bNodeTreeDeclaration::insert_socket(
    blender::StringRef name, const eNodeSocketDeclarationInOut in_out, const int index)
{
  if (!blender::IndexRange(sockets().size() + 1).contains(index)) {
    return nullptr;
  }

  bNodeSocketDeclaration *new_socket = MEM_cnew<bNodeSocketDeclaration>(__func__);
  new_socket->name = BLI_strdup(name.data());
  new_socket->in_out = in_out;

  blender::MutableSpan<bNodeSocketDeclaration *> old_sockets = sockets();
  sockets_num++;
  sockets_array = MEM_cnew_array<bNodeSocketDeclaration *>(sockets_num, __func__);
  sockets().take_front(index).copy_from(old_sockets.take_front(index));
  sockets().drop_front(index + 1).copy_from(old_sockets.drop_front(index));
  sockets()[index] = new_socket;

  if (!old_sockets.is_empty()) {
    MEM_freeN(old_sockets.data());
  }

  return new_socket;
}

bool bNodeTreeDeclaration::remove_socket(bNodeSocketDeclaration &socket_decl)
{
  const int index = socket_index(socket_decl);
  if (!sockets().index_range().contains(index)) {
    return false;
  }

  blender::MutableSpan<bNodeSocketDeclaration *> old_sockets = sockets();
  sockets_num--;
  sockets_array = MEM_cnew_array<bNodeSocketDeclaration *>(sockets_num, __func__);
  sockets().take_front(index).copy_from(old_sockets.take_front(index));
  sockets().drop_front(index).copy_from(old_sockets.drop_front(index + 1));

  MEM_SAFE_FREE(socket_decl.name);
  MEM_SAFE_FREE(socket_decl.description);
  /* Guaranteed not empty, contains at least the removed item */
  MEM_freeN(old_sockets.data());

  return true;
}

void bNodeTreeDeclaration::clear_sockets()
{
  for (bNodeSocketDeclaration *socket : sockets()) {
    MEM_SAFE_FREE(socket->name);
    MEM_SAFE_FREE(socket->description);
    MEM_SAFE_FREE(socket);
  }
  MEM_SAFE_FREE(sockets_array);
  sockets_num = 0;
  sockets_array = nullptr;
}

bool bNodeTreeDeclaration::move_socket(bNodeSocketDeclaration &socket_decl, const int new_index)
{
  const int old_index = socket_index(socket_decl);
  if (!sockets().index_range().contains(old_index) || !sockets().index_range().contains(new_index))
  {
    return false;
  }

  if (old_index == new_index) {
    /* Nothing changes. */
    return true;
  }
  else if (old_index < new_index) {
    const blender::Span<bNodeSocketDeclaration *> moved_sockets = sockets().slice(
        old_index + 1, new_index - old_index);
    bNodeSocketDeclaration *tmp = sockets()[old_index];
    std::copy(moved_sockets.begin(), moved_sockets.end(), sockets().drop_front(old_index).data());
    sockets()[new_index] = tmp;
  }
  else /* old_index > new_index */ {
    const blender::Span<bNodeSocketDeclaration *> moved_sockets = sockets().slice(
        new_index, old_index - new_index);
    bNodeSocketDeclaration *tmp = sockets()[old_index];
    std::copy_backward(
        moved_sockets.begin(), moved_sockets.end(), sockets().drop_front(old_index + 1).data());
    sockets()[new_index] = tmp;
  }

  return true;
}

int bNodeTreeDeclaration::panel_index(bNodePanel &panel) const
{
  return panels().first_index_try(&panel);
}

bNodePanel *bNodeTreeDeclaration::add_panel(blender::StringRef name)
{
  bNodePanel *new_panel = MEM_cnew<bNodePanel>(__func__);
  new_panel->name = BLI_strdup(name.data());

  blender::MutableSpan<bNodePanel *> old_panels = panels();
  panels_num++;
  panels_array = MEM_cnew_array<bNodePanel *>(panels_num, __func__);
  panels().drop_back(1).copy_from(old_panels);
  panels().last() = new_panel;

  if (!old_panels.is_empty()) {
    MEM_freeN(old_panels.data());
  }

  return new_panel;
}

bNodePanel *bNodeTreeDeclaration::insert_panel(blender::StringRef name, const int index)
{
  if (!blender::IndexRange(panels().size() + 1).contains(index)) {
    return nullptr;
  }

  bNodePanel *old_panel = MEM_cnew<bNodePanel>(__func__);
  old_panel->name = BLI_strdup(name.data());

  blender::MutableSpan<bNodePanel *> old_panels = panels();
  panels_num++;
  panels_array = MEM_cnew_array<bNodePanel *>(panels_num, __func__);
  panels().take_front(index).copy_from(old_panels.take_front(index));
  panels().drop_front(index + 1).copy_from(old_panels.drop_front(index));
  panels()[index] = old_panel;

  if (!old_panels.is_empty()) {
    MEM_freeN(old_panels.data());
  }

  return old_panel;
}

bool bNodeTreeDeclaration::remove_panel(bNodePanel &panel)
{
  const int index = panel_index(panel);
  if (!panels().index_range().contains(index)) {
    return false;
  }

  blender::MutableSpan<bNodePanel *> old_panels = panels();
  panels_num--;
  panels_array = MEM_cnew_array<bNodePanel *>(panels_num, __func__);
  panels().take_front(index).copy_from(old_panels.take_front(index));
  panels().drop_front(index).copy_from(old_panels.drop_front(index + 1));

  MEM_SAFE_FREE(panel.name);
  /* Guaranteed not empty, contains at least the removed item */
  MEM_freeN(old_panels.data());

  return true;
}

void bNodeTreeDeclaration::clear_panels()
{
  for (bNodePanel *panel : panels()) {
    MEM_SAFE_FREE(panel->name);
    MEM_SAFE_FREE(panel);
  }
  MEM_SAFE_FREE(panels_array);
  panels_num = 0;
  panels_array = nullptr;
}

bool bNodeTreeDeclaration::move_panel(bNodePanel &panel, const int new_index)
{
  const int old_index = panel_index(panel);
  if (!panels().index_range().contains(old_index) || !panels().index_range().contains(new_index)) {
    return false;
  }

  if (old_index == new_index) {
    /* Nothing changes. */
    return true;
  }
  else if (old_index < new_index) {
    const blender::Span<bNodePanel *> moved_panels = panels().slice(old_index + 1,
                                                                    new_index - old_index);
    bNodePanel *tmp = panels()[old_index];
    std::copy(moved_panels.begin(), moved_panels.end(), panels().drop_front(old_index).data());
    panels()[new_index] = tmp;
  }
  else /* old_index > new_index */ {
    const blender::Span<bNodePanel *> moved_panels = panels().slice(new_index,
                                                                    old_index - new_index);
    bNodePanel *tmp = panels()[old_index];
    std::copy_backward(
        moved_panels.begin(), moved_panels.end(), panels().drop_front(old_index + 1).data());
    panels()[new_index] = tmp;
  }

  return true;
}
