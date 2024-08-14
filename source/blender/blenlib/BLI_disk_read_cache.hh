/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <memory>

#include "BLI_any.hh"
#include "BLI_function_ref.hh"
#include "BLI_utildefines.h"

namespace blender::disk_read_cache {

class ReadKey {
  virtual ~ReadKey();

  virtual uint64_t hash() const = 0;
  virtual bool equal_to(const ReadKey &other) const = 0;
};

class ReadValue {
  virtual ~ReadValue();
};

const ReadValue *read(const ReadKey &key,
                      std::unique_ptr<ReadValue> (*read_fn)(const ReadKey &key));

}  // namespace blender::disk_read_cache
