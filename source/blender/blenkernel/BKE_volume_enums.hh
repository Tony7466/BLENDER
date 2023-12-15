/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

enum VolumeGridType : int8_t {
  VOLUME_GRID_UNKNOWN = 0,
  VOLUME_GRID_BOOLEAN,
  VOLUME_GRID_FLOAT,
  VOLUME_GRID_DOUBLE,
  VOLUME_GRID_INT,
  VOLUME_GRID_INT64,
  VOLUME_GRID_MASK,
  VOLUME_GRID_VECTOR_FLOAT,
  VOLUME_GRID_VECTOR_DOUBLE,
  VOLUME_GRID_VECTOR_INT,
  VOLUME_GRID_POINTS,
};

/* Describes how tree data in a volume grid was created. */
enum VolumeTreeSource {
  /* Tree data has been generated at runtime. */
  VOLUME_TREE_SOURCE_GENERATED,
  /* Tree is an empty placeholder for lazily loaded file data. */
  VOLUME_TREE_SOURCE_FILE_PLACEHOLDER,
  /* Tree data has been loaded from file. */
  VOLUME_TREE_SOURCE_FILE_LOADED,
  /* Tree is a simplified version of file data. */
  VOLUME_TREE_SOURCE_FILE_SIMPLIFIED,
};
