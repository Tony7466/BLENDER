/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

/* Volumetric read/write lambda functions - default implementations */
#ifndef VOLUME_READ_LAMBDA
#  define VOLUME_READ_LAMBDA(function_call) \
    auto volume_read_lambda_pass = [=](const int i) { return function_call; };
#  define VOLUME_WRITE_LAMBDA(function_call) \
    auto volume_write_lambda_pass = [=](const int i, VolumeStack entry) { function_call; };
#endif

/* Volume Stack
 *
 * This is an array of object/shared ID's that the current segment of the path
 * is inside of. */

/* Gets the volume stack priority of the given shader. */
ccl_device_inline int volume_stack_shader_priority(KernelGlobals kg, int shader)
{
  return kernel_data_fetch(shaders, (shader & SHADER_MASK)).volume_stack_priority;
}

/* Checks whether the given shader should be skipped
 * (if we're currently in a volume with a higher priority). */
ccl_device_inline int volume_stack_skip_shader(KernelGlobals kg, int shader, int active_priority)
{
  int priority = volume_stack_shader_priority(kg, shader);
  return (priority > 0) && (priority < active_priority);
}

/* Checks whether the given shader should be considered when iterating over all volumes at
 * the current point (during ray marching).
 * Skips volumes that are being masked by priority, as well as ones that don't actually have
 * a volume shader. */
ccl_device_inline bool volume_stack_volume_active(KernelGlobals kg, int shader, int active_priority)
{
  int flags = kernel_data_fetch(shaders, (shader & SHADER_MASK)).flags;
  return (flags & SD_HAS_VOLUME) && !volume_stack_skip_shader(kg, shader, active_priority);
}

/* Gets the currently active priority (as in, the highest priority of all volumes on the stack). */
template<typename StackReadOp>
ccl_device_inline int volume_stack_active_priority(KernelGlobals kg, StackReadOp stack_read)
{
  /* Since the stack is sorted by priority, just look at the first entry. */
  int shader = stack_read(0).shader;
  return (shader == SHADER_NONE) ? 0 : volume_stack_shader_priority(kg, shader);
}

/* Checks whether the current surface should be skipped, and sets the SD flag accordingly. */
template<typename StackReadOp>
ccl_device_inline void volume_stack_set_surface_priority(KernelGlobals kg, ccl_private ShaderData *sd, StackReadOp stack_read)
{
  int priority = volume_stack_shader_priority(kg, sd->shader);
  if (priority == 0) {
    return;
  }

  VolumeStack active = stack_read(0);
  if (active.shader == SHADER_NONE) {
    return;
  }

  int active_priority = volume_stack_shader_priority(kg, active.shader);
  if (priority < active_priority) {
    sd->flag |= SD_BSDF_PRIORIITY_MASKED;
  }
  else if (active_priority > 0) {
    if (sd->flag & SD_BACKFACING) {
      /* We're leaving the currently active volume, so the opposite medium is the next in line. */
      VolumeStack next = stack_read(1);
      sd->opposite_ior = (next.shader == SHADER_NONE)? 1.0f : next.ior;
    }
    else {
      /* We're entering a volume with higher priority, so the opposite medium is the currently highest one. */
      sd->opposite_ior = active.ior;
    }
  }
}

template<typename StackReadOp, typename StackWriteOp>
ccl_device void volume_stack_enter_exit(KernelGlobals kg,
                                        ccl_private const ShaderData *sd,
                                        const float ior,
                                        StackReadOp stack_read,
                                        StackWriteOp stack_write)
{
  /* todo: we should have some way for objects to indicate if they want the
   * world shader to work inside them. excluding it by default is problematic
   * because non-volume objects can't be assumed to be closed manifolds */

  const int priority = volume_stack_shader_priority(kg, sd->shader);
  if (!(sd->flag & SD_HAS_VOLUME) && priority == 0) {
    return;
  }

  if (sd->flag & SD_BACKFACING) {
    /* Exit volume object: remove from stack. */
    for (int i = 0;; i++) {
      VolumeStack entry = stack_read(i);
      if (entry.shader == SHADER_NONE) {
        break;
      }

      if (entry.object == sd->object && entry.shader == sd->shader) {
        /* Shift back next stack entries. */
        do {
          entry = stack_read(i + 1);
          stack_write(i, entry);
          i++;
        } while (entry.shader != SHADER_NONE);

        return;
      }
    }
  }
  else {
    /* Enter volume object: add to stack. */
    int i;
    int insert_pos = 0;
    for (i = 0;; i++) {
      VolumeStack entry = stack_read(i);
      if (entry.shader == SHADER_NONE) {
        break;
      }

      /* Already in the stack? then we have nothing to do. */
      if (entry.object == sd->object && entry.shader == sd->shader) {
        return;
      }

      /* Keep advancing the insert position until we reach entries with lower priority. */
      if (volume_stack_shader_priority(kg, entry.shader) > priority) {
        insert_pos = i + 1;
      }
    }

    /* If we exceed the stack limit, ignore. */
    if (i >= kernel_data.volume_stack_size - 1) {
      return;
    }

    /* Shift back stack entries starting at (and including) insert_pos. */
    for (int j = i; j >= insert_pos; j--) {
      stack_write(j + 1, stack_read(j));
    }

    /* Add entry at insert position. */
    const VolumeStack new_entry = {sd->object, sd->shader, ior};
    stack_write(insert_pos, new_entry);
  }
}

ccl_device void volume_stack_enter_exit(KernelGlobals kg,
                                        IntegratorState state,
                                        ccl_private const ShaderData *sd,
                                        const float ior)
{
  VOLUME_READ_LAMBDA(integrator_state_read_volume_stack(state, i))
  VOLUME_WRITE_LAMBDA(integrator_state_write_volume_stack(state, i, entry))
  volume_stack_enter_exit(kg, sd, ior, volume_read_lambda_pass, volume_write_lambda_pass);
}

ccl_device void shadow_volume_stack_enter_exit(KernelGlobals kg,
                                               IntegratorShadowState state,
                                               ccl_private const ShaderData *sd)
{
  VOLUME_READ_LAMBDA(integrator_state_read_shadow_volume_stack(state, i))
  VOLUME_WRITE_LAMBDA(integrator_state_write_shadow_volume_stack(state, i, entry))
  /* IOR does not matter in shadow stack, refractive interfaces block the shadow ray anyways. */
  volume_stack_enter_exit(kg, sd, 0.0f, volume_read_lambda_pass, volume_write_lambda_pass);
}

/* Clean stack after the last bounce.
 *
 * It is expected that all volumes are closed manifolds, so at the time when ray
 * hits nothing (for example, it is a last bounce which goes to environment) the
 * only expected volume in the stack is the world's one. All the rest volume
 * entries should have been exited already.
 *
 * This isn't always true because of ray intersection precision issues, which
 * could lead us to an infinite non-world volume in the stack, causing render
 * artifacts.
 *
 * Use this function after the last bounce to get rid of all volumes apart from
 * the world's one after the last bounce to avoid render artifacts.
 */
ccl_device_inline void volume_stack_clean(KernelGlobals kg, IntegratorState state)
{
  if (kernel_data.background.volume_shader != SHADER_NONE) {
    /* Keep the world's volume in stack. */
    INTEGRATOR_STATE_ARRAY_WRITE(state, volume_stack, 1, shader) = SHADER_NONE;
  }
  else {
    INTEGRATOR_STATE_ARRAY_WRITE(state, volume_stack, 0, shader) = SHADER_NONE;
  }
}

template<typename StackReadOp>
ccl_device float volume_stack_step_size(KernelGlobals kg, StackReadOp stack_read)
{
  float step_size = FLT_MAX;
  int active_priority = volume_stack_active_priority(kg, stack_read);

  for (int i = 0;; i++) {
    VolumeStack entry = stack_read(i);
    if (entry.shader == SHADER_NONE) {
      break;
    }
    if (!volume_stack_volume_active(kg, entry.shader, active_priority)) {
      continue;
    }

    int shader_flag = kernel_data_fetch(shaders, (entry.shader & SHADER_MASK)).flags;

    bool heterogeneous = false;

    if (shader_flag & SD_HETEROGENEOUS_VOLUME) {
      heterogeneous = true;
    }
    else if (shader_flag & SD_NEED_VOLUME_ATTRIBUTES) {
      /* We want to render world or objects without any volume grids
       * as homogeneous, but can only verify this at run-time since other
       * heterogeneous volume objects may be using the same shader. */
      int object = entry.object;
      if (object != OBJECT_NONE) {
        int object_flag = kernel_data_fetch(object_flag, object);
        if (object_flag & SD_OBJECT_HAS_VOLUME_ATTRIBUTES) {
          heterogeneous = true;
        }
      }
    }

    if (heterogeneous) {
      float object_step_size = object_volume_step_size(kg, entry.object);
      object_step_size *= kernel_data.integrator.volume_step_rate;
      step_size = fminf(object_step_size, step_size);
    }
  }

  return step_size;
}

typedef enum VolumeSampleMethod {
  VOLUME_SAMPLE_NONE = 0,
  VOLUME_SAMPLE_DISTANCE = (1 << 0),
  VOLUME_SAMPLE_EQUIANGULAR = (1 << 1),
  VOLUME_SAMPLE_MIS = (VOLUME_SAMPLE_DISTANCE | VOLUME_SAMPLE_EQUIANGULAR),
} VolumeSampleMethod;

ccl_device VolumeSampleMethod volume_stack_sample_method(KernelGlobals kg, IntegratorState state)
{
  VolumeSampleMethod method = VOLUME_SAMPLE_NONE;
  VOLUME_READ_LAMBDA(integrator_state_read_volume_stack(state, i))
  int active_priority = volume_stack_active_priority(kg, volume_read_lambda_pass);

  for (int i = 0;; i++) {
    VolumeStack entry = integrator_state_read_volume_stack(state, i);
    if (entry.shader == SHADER_NONE) {
      break;
    }
    if (!volume_stack_volume_active(kg, entry.shader, active_priority)) {
      continue;
    }

    int shader_flag = kernel_data_fetch(shaders, (entry.shader & SHADER_MASK)).flags;

    if (shader_flag & SD_VOLUME_MIS) {
      /* Multiple importance sampling. */
      return VOLUME_SAMPLE_MIS;
    }
    else if (shader_flag & SD_VOLUME_EQUIANGULAR) {
      /* Distance + equiangular sampling -> multiple importance sampling. */
      if (method == VOLUME_SAMPLE_DISTANCE) {
        return VOLUME_SAMPLE_MIS;
      }

      /* Only equiangular sampling. */
      method = VOLUME_SAMPLE_EQUIANGULAR;
    }
    else {
      /* Distance + equiangular sampling -> multiple importance sampling. */
      if (method == VOLUME_SAMPLE_EQUIANGULAR) {
        return VOLUME_SAMPLE_MIS;
      }

      /* Distance sampling only. */
      method = VOLUME_SAMPLE_DISTANCE;
    }
  }

  return method;
}

CCL_NAMESPACE_END
