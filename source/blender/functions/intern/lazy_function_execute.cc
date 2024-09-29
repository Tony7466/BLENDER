/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup fn
 */

#include "FN_lazy_function_execute.hh"

namespace blender::fn::lazy_function {

/* -------------------------------------------------------------------- */
/** \name BasicParams.
 * \{ */

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

void *BasicParams::try_get_input_data_ptr_impl(const Slot slot) const
{
  return inputs_[slot.main_index()].get();
}

void *BasicParams::try_get_input_data_ptr_or_request_impl(const Slot slot)
{
  void *value = inputs_[slot.main_index()].get();
  if (value == nullptr) {
    input_usages_[slot.main_index()] = ValueUsage::Used;
  }
  return value;
}

void *BasicParams::get_output_data_ptr_impl(const Slot slot)
{
  return outputs_[slot.main_index()].get();
}

void BasicParams::output_set_impl(const Slot slot)
{
  set_outputs_[slot.main_index()] = true;
}

bool BasicParams::output_was_set_impl(const Slot slot) const
{
  return set_outputs_[slot.main_index()];
}

ValueUsage BasicParams::get_output_usage_impl(const Slot slot) const
{
  return output_usages_[slot.main_index()];
}

void BasicParams::set_input_unused_impl(const Slot slot)
{
  input_usages_[slot.main_index()] = ValueUsage::Unused;
}

bool BasicParams::try_enable_multi_threading_impl()
{
  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name RemappedParams.
 * \{ */

RemappedParams::RemappedParams(const LazyFunction &fn,
                               Params &base_params,
                               const Span<int> input_map,
                               const Span<int> output_map,
                               bool &multi_threading_enabled)
    : Params(fn, multi_threading_enabled),
      base_params_(base_params),
      input_map_(input_map),
      output_map_(output_map),
      multi_threading_enabled_(multi_threading_enabled)
{
}

void *RemappedParams::try_get_input_data_ptr_impl(const Slot slot) const
{
  return base_params_.try_get_input_data_ptr(input_map_[slot.main_index()]);
}

void *RemappedParams::try_get_input_data_ptr_or_request_impl(const Slot slot)
{
  return base_params_.try_get_input_data_ptr_or_request(input_map_[slot.main_index()]);
}

void *RemappedParams::get_output_data_ptr_impl(const Slot slot)
{
  return base_params_.get_output_data_ptr(output_map_[slot.main_index()]);
}

void RemappedParams::output_set_impl(const Slot slot)
{
  return base_params_.output_set(output_map_[slot.main_index()]);
}

bool RemappedParams::output_was_set_impl(const Slot slot) const
{
  return base_params_.output_was_set(output_map_[slot.main_index()]);
}

lf::ValueUsage RemappedParams::get_output_usage_impl(const Slot slot) const
{
  return base_params_.get_output_usage(output_map_[slot.main_index()]);
}

void RemappedParams::set_input_unused_impl(const Slot slot)
{
  return base_params_.set_input_unused(input_map_[slot.main_index()]);
}

bool RemappedParams::try_enable_multi_threading_impl()
{
  if (multi_threading_enabled_) {
    return true;
  }
  if (base_params_.try_enable_multi_threading()) {
    multi_threading_enabled_ = true;
    return true;
  }
  return false;
}

/** \} */

}  // namespace blender::fn::lazy_function
