/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_MultiThreadedOperation.h"

namespace blender::compositor {

class ConvertBaseOperation : public MultiThreadedOperation {
 protected:
  SocketReader *input_operation_;

 public:
  ConvertBaseOperation();

  void init_execution() override;
  void deinit_execution() override;

  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) final;

 protected:
  virtual void hash_output_params() override;
  virtual void update_memory_buffer_partial(BuffersIterator<float> &it) = 0;
};

class ConvertValueToColorOperation : public ConvertBaseOperation {
 public:
  ConvertValueToColorOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertColorToValueOperation : public ConvertBaseOperation {
 public:
  ConvertColorToValueOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertColorToBWOperation : public ConvertBaseOperation {
 public:
  ConvertColorToBWOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertColorToVectorOperation : public ConvertBaseOperation {
 public:
  ConvertColorToVectorOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertValueToVectorOperation : public ConvertBaseOperation {
 public:
  ConvertValueToVectorOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertVectorToColorOperation : public ConvertBaseOperation {
 public:
  ConvertVectorToColorOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertVectorToValueOperation : public ConvertBaseOperation {
 public:
  ConvertVectorToValueOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertRGBToYCCOperation : public ConvertBaseOperation {
 private:
  /** YCbCr mode (JPEG, ITU601, ITU709) */
  int mode_;

 public:
  ConvertRGBToYCCOperation();

  /** Set the YCC mode */
  void set_mode(int mode);

 protected:
  void hash_output_params() override;
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertYCCToRGBOperation : public ConvertBaseOperation {
 private:
  /** YCbCr mode (JPEG, ITU601, ITU709) */
  int mode_;

 public:
  ConvertYCCToRGBOperation();

  /** Set the YCC mode */
  void set_mode(int mode);

 protected:
  void hash_output_params() override;
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertRGBToYUVOperation : public ConvertBaseOperation {
 public:
  ConvertRGBToYUVOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertYUVToRGBOperation : public ConvertBaseOperation {
 public:
  ConvertYUVToRGBOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertRGBToHSVOperation : public ConvertBaseOperation {
 public:
  ConvertRGBToHSVOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertHSVToRGBOperation : public ConvertBaseOperation {
 public:
  ConvertHSVToRGBOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertRGBToHSLOperation : public ConvertBaseOperation {
 public:
  ConvertRGBToHSLOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertHSLToRGBOperation : public ConvertBaseOperation {
 public:
  ConvertHSLToRGBOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertPremulToStraightOperation : public ConvertBaseOperation {
 public:
  ConvertPremulToStraightOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class ConvertStraightToPremulOperation : public ConvertBaseOperation {
 public:
  ConvertStraightToPremulOperation();

 protected:
  void update_memory_buffer_partial(BuffersIterator<float> &it) override;
};

class SeparateChannelOperation : public MultiThreadedOperation {
 private:
  SocketReader *input_operation_;
  int channel_;

 public:
  SeparateChannelOperation();

  void init_execution() override;
  void deinit_execution() override;

  void set_channel(int channel)
  {
    channel_ = channel;
  }

  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;
};

class CombineChannelsOperation : public MultiThreadedOperation {
 private:
  SocketReader *input_channel1_operation_;
  SocketReader *input_channel2_operation_;
  SocketReader *input_channel3_operation_;
  SocketReader *input_channel4_operation_;

 public:
  CombineChannelsOperation();

  void init_execution() override;
  void deinit_execution() override;

  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;
};

}  // namespace blender::compositor
