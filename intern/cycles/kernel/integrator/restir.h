#ifndef RESTIR_H_
#define RESTIR_H_

CCL_NAMESPACE_BEGIN

ccl_device void integrator_restir(KernelGlobals kg,
                                  IntegratorShadowState state,
                                  ccl_global float *ccl_restrict render_buffer)
{
  PROFILING_INIT(kg, PROFILING_SHADE_RESTIR);
  film_write_direct_light(kg, state, render_buffer);
  integrator_shadow_path_terminate(kg, state, DEVICE_KERNEL_INTEGRATOR_RESTIR);
  return;
}

CCL_NAMESPACE_END

#endif  // RESTIR_H_
