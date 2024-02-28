/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_MultiThreadedOperation.h"

namespace blender::compositor {

class DilateErodeThresholdOperation : public MultiThreadedOperation {
 public:
  struct PixelData;

 private:
  /**
   * Cached reference to the input_program
   */
  SocketReader *input_program_;

  float distance_;
  float switch_;
  float inset_;

  /**
   * determines the area of interest to track pixels
   * keep this one as small as possible for speed gain.
   */
  int scope_;

 public:
  /* DilateErode Distance Threshold */
  DilateErodeThresholdOperation();

  void init_data() override;
  /**
   * Initialize the execution
   */
  void init_execution() override;

  /**
   * Deinitialize the execution
   */
  void deinit_execution() override;

  void set_distance(float distance)
  {
    distance_ = distance;
  }
  void set_switch(float sw)
  {
    switch_ = sw;
  }
  void set_inset(float inset)
  {
    inset_ = inset;
  }

  void get_area_of_interest(int input_idx, const rcti &output_area, rcti &r_input_area) override;
  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;
};

class DilateDistanceOperation : public MultiThreadedOperation {
 public:
  struct PixelData;

 protected:
  /**
   * Cached reference to the input_program
   */
  SocketReader *input_program_;
  float distance_;
  int scope_;

 public:
  /* Dilate Distance. */
  DilateDistanceOperation();

  void init_data() override;
  /**
   * Initialize the execution
   */
  void init_execution() override;

  /**
   * Deinitialize the execution
   */
  void deinit_execution() override;

  void set_distance(float distance)
  {
    distance_ = distance;
  }
  void get_area_of_interest(int input_idx, const rcti &output_area, rcti &r_input_area) final;
  virtual void update_memory_buffer_partial(MemoryBuffer *output,
                                            const rcti &area,
                                            Span<MemoryBuffer *> inputs) override;
};

class ErodeDistanceOperation : public DilateDistanceOperation {
 public:
  /* Erode Distance */
  ErodeDistanceOperation();

  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;
};

class DilateStepOperation : public MultiThreadedOperation {
 protected:
  /**
   * Cached reference to the input_program
   */
  SocketReader *input_program_;

  int iterations_;

 public:
  /* Dilate step */
  DilateStepOperation();

  /**
   * Initialize the execution
   */
  void init_execution() override;

  /**
   * Deinitialize the execution
   */
  void deinit_execution() override;

  void set_iterations(int iterations)
  {
    iterations_ = iterations;
  }

  void get_area_of_interest(int input_idx, const rcti &output_area, rcti &r_input_area) final;
  virtual void update_memory_buffer_partial(MemoryBuffer *output,
                                            const rcti &area,
                                            Span<MemoryBuffer *> inputs) override;
};

class ErodeStepOperation : public DilateStepOperation {
 public:
  /** Erode step. */
  ErodeStepOperation();

  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;
};

}  // namespace blender::compositor
