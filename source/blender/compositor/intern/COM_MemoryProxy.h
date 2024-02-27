/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifdef WITH_CXX_GUARDEDALLOC
#  include "MEM_guardedalloc.h"
#endif

#include "COM_defines.h"

namespace blender::compositor {

class MemoryBuffer;
class WriteBufferOperation;

/**
 * \brief A MemoryProxy is a unique identifier for a memory buffer.
 * A single MemoryProxy is used among all chunks of the same buffer,
 * the MemoryBuffer only stores the data of a single chunk.
 * \ingroup Memory
 */
class MemoryProxy {
 private:
  /**
   * \brief reference to the output operation of the executiongroup
   */
  WriteBufferOperation *write_buffer_operation_;

  /**
   * \brief the allocated memory
   */
  MemoryBuffer *buffer_;

  /**
   * \brief datatype of this MemoryProxy
   */
  DataType datatype_;

 public:
  MemoryProxy(DataType type);

  /**
   * \brief set the WriteBufferOperation that is responsible for writing to this MemoryProxy
   * \param operation:
   */
  void set_write_buffer_operation(WriteBufferOperation *operation)
  {
    write_buffer_operation_ = operation;
  }

  /**
   * \brief get the WriteBufferOperation that is responsible for writing to this MemoryProxy
   * \return WriteBufferOperation
   */
  WriteBufferOperation *get_write_buffer_operation() const
  {
    return write_buffer_operation_;
  }

  /**
   * \brief allocate memory of size width x height
   */
  void allocate(unsigned int width, unsigned int height);

  /**
   * \brief free the allocated memory
   */
  void free();

  /**
   * \brief get the allocated memory
   */
  inline MemoryBuffer *get_buffer()
  {
    return buffer_;
  }

  inline DataType get_data_type()
  {
    return datatype_;
  }

#ifdef WITH_CXX_GUARDEDALLOC
  MEM_CXX_CLASS_ALLOC_FUNCS("COM:MemoryProxy")
#endif
};

}  // namespace blender::compositor
