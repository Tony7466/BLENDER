/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2021-2022 Blender Foundation */

// clang-format off

/* Open the Metal kernel context class
 * Necessary to access resource bindings */
class MetalKernelContext {
  public:
    constant KernelParamsMetal &launch_params_metal;
    constant MetalAncillaries *metal_ancillaries;

    MetalKernelContext(constant KernelParamsMetal &_launch_params_metal, constant MetalAncillaries * _metal_ancillaries)
    : launch_params_metal(_launch_params_metal), metal_ancillaries(_metal_ancillaries)
    {}

    MetalKernelContext(constant KernelParamsMetal &_launch_params_metal)
    : launch_params_metal(_launch_params_metal)
    {}

#ifdef __KERNEL_METAL_BUFFER_TEXTURES__
    template<typename T> ccl_device_forceinline T tex_fetch(device void* data, int64_t index)
    {
      return reinterpret_cast<ccl_global T *>(data)[index];
    }
    
    ccl_device_inline int svm_image_texture_wrap_periodic(int x, int width)
    {
      x %= width;
      if (x < 0)
        x += width;
      return x;
    }
    
    ccl_device_inline int svm_image_texture_wrap_clamp(int x, int width)
    {
      return clamp(x, 0, width - 1);
    }
    
    ccl_device_inline int svm_image_texture_wrap_mirror(int x, int width)
    {
      const int m = abs(x + (x < 0)) % (2 * width);
      if (m >= width)
        return 2 * width - m - 1;
      return m;
    }
    
    ccl_device_inline float4 svm_image_texture_read(device const TextureInfo &info, device void *data, int x, int y, int z)
    {
      const int64_t data_offset = int64_t(x) + int64_t(info.width) * int64_t(y) + int64_t(info.width) * int64_t(info.height) * int64_t(z);
      const int texture_type = info.data_type;
    
      /* Float4 */
      if (texture_type == IMAGE_DATA_TYPE_FLOAT4) {
        return tex_fetch<float4>(data, data_offset);
      }
      /* Byte4 */
      else if (texture_type == IMAGE_DATA_TYPE_BYTE4) {
        uchar4 r = tex_fetch<uchar4>(data, data_offset);
        float f = 1.0f / 255.0f;
        return make_float4(r.x * f, r.y * f, r.z * f, r.w * f);
      }
      /* Ushort4 */
      else if (texture_type == IMAGE_DATA_TYPE_USHORT4) {
        ushort4 r = tex_fetch<ushort4>(data, data_offset);
        float f = 1.0f / 65535.f;
        return make_float4(r.x * f, r.y * f, r.z * f, r.w * f);
      }
      /* Float */
      else if (texture_type == IMAGE_DATA_TYPE_FLOAT) {
        float f = tex_fetch<float>(data, data_offset);
        return make_float4(f, f, f, 1.0f);
      }
      /* UShort */
      else if (texture_type == IMAGE_DATA_TYPE_USHORT) {
        ushort r = tex_fetch<ushort>(data, data_offset);
        float f = r * (1.0f / 65535.0f);
        return make_float4(f, f, f, 1.0f);
      }
      else if (texture_type == IMAGE_DATA_TYPE_HALF) {
        float f = tex_fetch<half>(data, data_offset);
        return make_float4(f, f, f, 1.0f);
      }
      else if (texture_type == IMAGE_DATA_TYPE_HALF4) {
        half4 r = tex_fetch<half4>(data, data_offset);
        return make_float4(r.x, r.y, r.z, r.w);
      }
      /* Byte */
      else {
        uchar r = tex_fetch<uchar>(data, data_offset);
        float f = r * (1.0f / 255.0f);
        return make_float4(f, f, f, 1.0f);
      }
    }

    ccl_device_inline float4 svm_image_texture_read_2d(device const TextureInfo &info, device void *data, int x, int y)
    {
      /* Wrap */
      if (info.extension == EXTENSION_REPEAT) {
        x = svm_image_texture_wrap_periodic(x, info.width);
        y = svm_image_texture_wrap_periodic(y, info.height);
      }
      else if (info.extension == EXTENSION_EXTEND) {
        x = svm_image_texture_wrap_clamp(x, info.width);
        y = svm_image_texture_wrap_clamp(y, info.height);
      }
      else if (info.extension == EXTENSION_MIRROR) {
        x = svm_image_texture_wrap_mirror(x, info.width);
        y = svm_image_texture_wrap_mirror(y, info.height);
      }
      else {
        if (x < 0 || x >= info.width || y < 0 || y >= info.height) {
          return make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        }
      }
  
      return svm_image_texture_read(info, data, x, y, 0);
    }
  
    ccl_device_inline float4 svm_image_texture_read_3d(device const TextureInfo &info, device void *data, int x, int y, int z)
    {
      /* Wrap */
      if (info.extension == EXTENSION_REPEAT) {
        x = svm_image_texture_wrap_periodic(x, info.width);
        y = svm_image_texture_wrap_periodic(y, info.height);
        z = svm_image_texture_wrap_periodic(z, info.depth);
      }
      else if (info.extension == EXTENSION_EXTEND) {
        x = svm_image_texture_wrap_clamp(x, info.width);
        y = svm_image_texture_wrap_clamp(y, info.height);
        z = svm_image_texture_wrap_clamp(z, info.depth);
      }
      else if (info.extension == EXTENSION_MIRROR) {
        x = svm_image_texture_wrap_mirror(x, info.width);
        y = svm_image_texture_wrap_mirror(y, info.height);
        z = svm_image_texture_wrap_mirror(z, info.depth);
      }
      else {
        if (x < 0 || x >= info.width || y < 0 || y >= info.height || z < 0 || z >= info.depth) {
          return make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        }
      }
  
      return svm_image_texture_read(info, data, x, y, z);
    }
  
    static float svm_image_texture_frac(float x, thread int *ix)
    {
      int i = float_to_int(x) - ((x < 0.0f) ? 1 : 0);
      *ix = i;
      return x - (float)i;
    }
  
    #define SET_CUBIC_SPLINE_WEIGHTS(u, t) \
      { \
        u[0] = (((-1.0f / 6.0f) * t + 0.5f) * t - 0.5f) * t + (1.0f / 6.0f); \
        u[1] = ((0.5f * t - 1.0f) * t) * t + (2.0f / 3.0f); \
        u[2] = ((-0.5f * t + 0.5f) * t + 0.5f) * t + (1.0f / 6.0f); \
        u[3] = (1.0f / 6.0f) * t * t * t; \
      } \
      (void)0

#endif /* __KERNEL_METAL_BUFFER_TEXTURES__ */

    ccl_device float4 kernel_tex_image_interp(KernelGlobals kg, int tex_id, float x, float y)
    {
      device const TextureInfo &info = kernel_data_fetch(texture_info, tex_id);
      
      const uint tid(info.data);
      const uint sid(info.data >> 32);
#ifndef __KERNEL_METAL_BUFFER_TEXTURES__
      return metal_ancillaries->textures_2d[tid].tex.sample(metal_samplers[sid], make_float2(x,y));
#else
      if (sid < 256) {
        return metal_ancillaries->textures_2d[tid].tex.sample(metal_samplers[sid], make_float2(x,y));
      }
      device void* data = metal_ancillaries->texture_buffers[tid].data;
  
      if (info.interpolation == INTERPOLATION_CLOSEST) {
        /* Closest interpolation. */
        int ix, iy;
        svm_image_texture_frac(x * info.width, &ix);
        svm_image_texture_frac(y * info.height, &iy);
  
        return svm_image_texture_read_2d(info, data,  ix, iy);
      }
      else {//if (info.interpolation == INTERPOLATION_LINEAR) {
        /* Bilinear interpolation. */
        int ix, iy;
        float tx = svm_image_texture_frac(x * info.width - 0.5f, &ix);
        float ty = svm_image_texture_frac(y * info.height - 0.5f, &iy);
  
        float4 r;
        r = (1.0f - ty) * (1.0f - tx) * svm_image_texture_read_2d(info, data,  ix, iy);
        r += (1.0f - ty) * tx * svm_image_texture_read_2d(info, data,  ix + 1, iy);
        r += ty * (1.0f - tx) * svm_image_texture_read_2d(info, data,  ix, iy + 1);
        r += ty * tx * svm_image_texture_read_2d(info, data,  ix + 1, iy + 1);
        return r;
      }
#if 0
      else {
        /* Bicubic interpolation. */
        int ix, iy;
        float tx = svm_image_texture_frac(x * info.width - 0.5f, &ix);
        float ty = svm_image_texture_frac(y * info.height - 0.5f, &iy);
  
        float u[4], v[4];
        SET_CUBIC_SPLINE_WEIGHTS(u, tx);
        SET_CUBIC_SPLINE_WEIGHTS(v, ty);
  
        float4 r = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
  
        for (int y = 0; y < 4; y++) {
          for (int x = 0; x < 4; x++) {
            float weight = u[x] * v[y];
            r += weight * svm_image_texture_read_2d(info, data, ix + x - 1, iy + y - 1);
          }
        }
        return r;
      }
 #endif
 #endif
    }
    
    ccl_device float4 kernel_tex_image_interp_3d(KernelGlobals kg, int tex_id, float3 P, int interp)
    {
      device const TextureInfo &info = kernel_data_fetch(texture_info, tex_id);
      
      const uint tid(info.data);
      const uint sid(info.data >> 32);

#ifndef __KERNEL_METAL_BUFFER_TEXTURES__
      return metal_ancillaries->textures_3d[tid].tex.sample(metal_samplers[sid], P);
#else
      if (sid < 256) {
        return metal_ancillaries->textures_3d[tid].tex.sample(metal_samplers[sid], P);
      }
      device void* data = metal_ancillaries->texture_buffers[tid].data;
  
      if (info.use_transform_3d) {
        Transform tfm = info.transform_3d;
        P = transform_point(&tfm, P);
      }
  
      float x = P.x;
      float y = P.y;
      float z = P.z;
  
      uint interpolation = (interp == INTERPOLATION_NONE) ? info.interpolation : interp;
  
#if 0//#ifdef WITH_NANOVDB
      if (info.data_type == IMAGE_DATA_TYPE_NANOVDB_FLOAT) {
        return NanoVDBInterpolator<float>::interp_3d(info, x, y, z, interpolation);
      }
      else if (info.data_type == IMAGE_DATA_TYPE_NANOVDB_FLOAT3) {
        return NanoVDBInterpolator<nanovdb::Vec3f>::interp_3d(info, x, y, z, interpolation);
      }
      else if (info.data_type == IMAGE_DATA_TYPE_NANOVDB_FPN) {
        return NanoVDBInterpolator<nanovdb::FpN>::interp_3d(info, x, y, z, interpolation);
      }
      else if (info.data_type == IMAGE_DATA_TYPE_NANOVDB_FP16) {
        return NanoVDBInterpolator<nanovdb::Fp16>::interp_3d(info, x, y, z, interpolation);
      }
#else
      if (info.data_type == IMAGE_DATA_TYPE_NANOVDB_FLOAT ||
          info.data_type == IMAGE_DATA_TYPE_NANOVDB_FLOAT3 ||
          info.data_type == IMAGE_DATA_TYPE_NANOVDB_FPN ||
          info.data_type == IMAGE_DATA_TYPE_NANOVDB_FP16) {
        return make_float4(
            TEX_IMAGE_MISSING_R, TEX_IMAGE_MISSING_G, TEX_IMAGE_MISSING_B, TEX_IMAGE_MISSING_A);
      }
#endif
      else {
        x *= info.width;
        y *= info.height;
        z *= info.depth;
      }
  
      if (interpolation == INTERPOLATION_CLOSEST) {
        /* Closest interpolation. */
        int ix, iy, iz;
        svm_image_texture_frac(x, &ix);
        svm_image_texture_frac(y, &iy);
        svm_image_texture_frac(z, &iz);
  
        return svm_image_texture_read_3d(info, data,  ix, iy, iz);
      }
      else {// if (interpolation == INTERPOLATION_LINEAR) {
        /* Trilinear interpolation. */
        int ix, iy, iz;
        float tx = svm_image_texture_frac(x - 0.5f, &ix);
        float ty = svm_image_texture_frac(y - 0.5f, &iy);
        float tz = svm_image_texture_frac(z - 0.5f, &iz);
  
        float4 r;
        r = (1.0f - tz) * (1.0f - ty) * (1.0f - tx) * svm_image_texture_read_3d(info, data,  ix, iy, iz);
        r += (1.0f - tz) * (1.0f - ty) * tx * svm_image_texture_read_3d(info, data,  ix + 1, iy, iz);
        r += (1.0f - tz) * ty * (1.0f - tx) * svm_image_texture_read_3d(info, data,  ix, iy + 1, iz);
        r += (1.0f - tz) * ty * tx * svm_image_texture_read_3d(info, data,  ix + 1, iy + 1, iz);
  
        r += tz * (1.0f - ty) * (1.0f - tx) * svm_image_texture_read_3d(info, data,  ix, iy, iz + 1);
        r += tz * (1.0f - ty) * tx * svm_image_texture_read_3d(info, data,  ix + 1, iy, iz + 1);
        r += tz * ty * (1.0f - tx) * svm_image_texture_read_3d(info, data,  ix, iy + 1, iz + 1);
        r += tz * ty * tx * svm_image_texture_read_3d(info, data,  ix + 1, iy + 1, iz + 1);
        return r;
      }
#if 0
      else {
        /* Tri-cubic interpolation. */
        int ix, iy, iz;
        float tx = svm_image_texture_frac(x - 0.5f, &ix);
        float ty = svm_image_texture_frac(y - 0.5f, &iy);
        float tz = svm_image_texture_frac(z - 0.5f, &iz);
  
        float u[4], v[4], w[4];
        SET_CUBIC_SPLINE_WEIGHTS(u, tx);
        SET_CUBIC_SPLINE_WEIGHTS(v, ty);
        SET_CUBIC_SPLINE_WEIGHTS(w, tz);
  
        float4 r = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
  
        for (int z = 0; z < 4; z++) {
          for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
              float weight = u[x] * v[y] * w[z];
              r += weight * svm_image_texture_read_3d(info, data, ix + x - 1, iy + y - 1, iz + z - 1);
            }
          }
        }
        return r;
      }
  #endif
  #endif
  }

  // clang-format on
