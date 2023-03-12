/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup fn
 */

#include "FN_lazy_function_execute.hh"

namespace blender::fn::lazy_function {

BasicParams::BasicParams(const LazyFunction &fn,
                         const Span<GMutablePointer> inputs,
                         const Span<GMutablePointer> outputs,
                         MutableSpan<std::optional<ValueUsage>> input_usages,
                         Span<ValueUsage> output_usages,
                         MutableSpan<bool> set_outputs)
    : Params(fn, true),
      inputs_(inputs),
      outputs_(outputs),
      input_usages_(input_usages),
      output_usages_(output_usages),
      set_outputs_(set_outputs)
{
}

void BasicParams::try_get_input_data_ptr_impl(const Span<int> indices,
                                              MutableSpan<void *> r_data) const
{
  for (const int i : indices.index_range()) {
    const int index = indices[i];
    r_data[i] = inputs_[index].get();
  }
}

void BasicParams::try_get_input_data_ptr_or_request_impl(const Span<int> indices,
                                                         MutableSpan<void *> r_data)
{
  for (const int i : indices.index_range()) {
    const int index = indices[i];
    void *value = inputs_[index].get();
    if (value == nullptr) {
      input_usages_[index] = ValueUsage::Used;
    }
    r_data[i] = value;
  }
}

void BasicParams::get_output_data_ptr_impl(const Span<int> indices, MutableSpan<void *> r_data)
{
  for (const int i : indices.index_range()) {
    const int index = indices[i];
    r_data[i] = outputs_[index].get();
  }
}

void BasicParams::output_set_impl(const Span<int> indices)
{
  for (const int index : indices) {
    set_outputs_[index] = true;
  }
}

void BasicParams::output_was_set_impl(const Span<int> indices, MutableSpan<bool> r_result) const
{
  for (const int i : indices.index_range()) {
    const int index = indices[i];
    r_result[i] = set_outputs_[index];
  }
}

void BasicParams::get_output_usage_impl(const Span<int> indices,
                                        MutableSpan<ValueUsage> r_result) const
{
  for (const int i : indices.index_range()) {
    const int index = indices[i];
    r_result[i] = output_usages_[index];
  }
}

void BasicParams::set_input_unused_impl(const int index)
{
  input_usages_[index] = ValueUsage::Unused;
}

bool BasicParams::try_enable_multi_threading_impl()
{
  return true;
}

}  // namespace blender::fn::lazy_function
