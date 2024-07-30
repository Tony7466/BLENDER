/* SPDX-FileCopyrightText: 2022-2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "WM_api.hh"

/* Required to avoid namespace clash between WM and Apple frameworks */
void *create_system_gpu_context()
{
  void *system_gpu_context = WM_system_gpu_context_create();
  return system_gpu_context;
}

