#pragma once

#define HIPRT_PUBLIC_DEVICE_H
#define HIPRT_DEVICE __device__

#if __gfx900__ || __gfx906__ || __gfx908__ || __gfx90a__
#  define WARP_THREADS 64
#else
#  define WARP_THREADS 32
#endif

#include "kernel/device/hiprt/hiprt_types.h"

/** \brief A stack using (slow) local memory internally.
 *
 */
class HIPRT_ALIGN(32) hiprtPrivateStack {
 public:
  static constexpr u32 StackSize = 64;
  HIPRT_DEVICE hiprtPrivateStack() : m_top(0u)
  {
  }

  HIPRT_DEVICE int pop()
  {
    return m_stackBuffer[--m_top];
  }
  HIPRT_DEVICE void push(int val)
  {
    m_stackBuffer[m_top++] = val;
  }
  HIPRT_DEVICE bool empty() const
  {
    return m_top == 0u;
  }
  HIPRT_DEVICE int vacancy() const
  {
    return StackSize - m_top;
  }
  HIPRT_DEVICE void reset()
  {
    m_top = 0u;
  }

 private:
  int m_stackBuffer[StackSize];
  u32 m_top;
};

/** \brief A stack using both (fast) shared memory and (slow) global memory.
 *
 * The stack uses shared memory if there is enough space.
 * Otherwise, it uses global memory as a backup.
 */
class HIPRT_ALIGN(32) hiprtGlobalStack {
 public:
  static constexpr u32 Stride = WARP_THREADS;
  static constexpr u32 LogStride = HIPRT_LOG(Stride);

  HIPRT_DEVICE
  hiprtGlobalStack(int *globalStackBuffer,
                   u32 globalStackSize,
                   int *sharedStackBuffer = nullptr,
                   u32 sharedStackSize = 0u);

  HIPRT_DEVICE int pop();

  HIPRT_DEVICE void push(int val);

  HIPRT_DEVICE int vacancy() const;

  HIPRT_DEVICE bool empty() const;

  HIPRT_DEVICE void reset();

 private:
  int *m_globalStackBuffer;
  int *m_sharedStackBuffer;
  const u32 m_globalStackSize;
  const u32 m_sharedStackSize;
  int m_globalIndex;
  int m_sharedIndex;
  u32 m_sharedCount;
};

/** \brief A traversal object for finding the closest hit with hiprtGeometry containing triangles.
 *
 * It uses a private stack with size 64 internally.
 */
template<hiprtPrimitiveNodeType PrimitiveNodeType, hiprtTraversalType TraversalType>
class hiprtGeomTraversal_impl;

class hiprtGeomTraversalClosest {
 public:
  HIPRT_DEVICE hiprtGeomTraversalClosest(hiprtGeometry geom,
                                         const hiprtRay &ray,
                                         hiprtTraversalHint hint = hiprtTraversalHintDefault,
                                         void *payload = nullptr,
                                         hiprtFuncTable funcTable = nullptr,
                                         u32 rayType = 0);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtGeomTraversal_impl<hiprtTriangleNode, hiprtTraversalTerminateAtClosestHit>,
             SIZE_GEOM_TRAVERSAL_PRIVATE_STACK,
             ALIGNMENT_GEOM_TRAVERSAL_PRIVATE_STACK>
      m_impl;
};

/** \brief A traversal object for finding the any hit with hiprtGeometry containing triangles.
 *
 * It uses a private stack with size 64 internally.
 */
class hiprtGeomTraversalAnyHit {
 public:
  HIPRT_DEVICE hiprtGeomTraversalAnyHit(hiprtGeometry geom,
                                        const hiprtRay &ray,
                                        hiprtTraversalHint hint = hiprtTraversalHintDefault,
                                        void *payload = nullptr,
                                        hiprtFuncTable funcTable = nullptr,
                                        u32 rayType = 0);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtGeomTraversal_impl<hiprtTriangleNode, hiprtTraversalTerminateAtAnyHit>,
             SIZE_GEOM_TRAVERSAL_PRIVATE_STACK,
             ALIGNMENT_GEOM_TRAVERSAL_PRIVATE_STACK>
      m_impl;
};

/** \brief A traversal object for finding the closest hit with hiprtGeometry containing custom
 * primitives.
 *
 * It uses a private stack with size 64 internally.
 */
class hiprtGeomCustomTraversalClosest {
 public:
  HIPRT_DEVICE hiprtGeomCustomTraversalClosest(hiprtGeometry geom,
                                               const hiprtRay &ray,
                                               hiprtTraversalHint hint = hiprtTraversalHintDefault,
                                               void *payload = nullptr,
                                               hiprtFuncTable funcTable = nullptr,
                                               u32 rayType = 0);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtGeomTraversal_impl<hiprtCustomNode, hiprtTraversalTerminateAtClosestHit>,
             SIZE_GEOM_TRAVERSAL_PRIVATE_STACK,
             ALIGNMENT_GEOM_TRAVERSAL_PRIVATE_STACK>
      m_impl;
};

/** \brief A traversal object for finding the any hit with hiprtGeometry containing custom
 * primitives.
 *
 * It uses a private stack with size 64 internally.
 */
class hiprtGeomCustomTraversalAnyHit {
 public:
  HIPRT_DEVICE hiprtGeomCustomTraversalAnyHit(hiprtGeometry geom,
                                              const hiprtRay &ray,
                                              hiprtTraversalHint hint = hiprtTraversalHintDefault,
                                              void *payload = nullptr,
                                              hiprtFuncTable funcTable = nullptr,
                                              u32 rayType = 0);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtGeomTraversal_impl<hiprtCustomNode, hiprtTraversalTerminateAtAnyHit>,
             SIZE_GEOM_TRAVERSAL_PRIVATE_STACK,
             ALIGNMENT_GEOM_TRAVERSAL_PRIVATE_STACK>
      m_impl;
};

/** \brief A traversal object for finding the closest hit with hiprtScene.
 *
 * It uses a private stack with size 64 internally.
 */
template<hiprtTraversalType TraversalType> class hiprtSceneTraversal_impl;

class hiprtSceneTraversalClosest {
 public:
  HIPRT_DEVICE hiprtSceneTraversalClosest(hiprtScene scene,
                                          const hiprtRay &ray,
                                          hiprtRayMask mask = hiprtFullRayMask,
                                          hiprtTraversalHint hint = hiprtTraversalHintDefault,
                                          void *payload = nullptr,
                                          hiprtFuncTable funcTable = nullptr,
                                          u32 rayType = 0,
                                          float time = 0.0f);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtSceneTraversal_impl<hiprtTraversalTerminateAtClosestHit>,
             SIZE_SCENE_TRAVERSAL_PRIVATE_STACK,
             ALIGNMENT_SCENE_TRAVERSAL_PRIVATE_STACK>
      m_impl;
};

/** \brief A traversal object for finding the any hit with hiprtScene.
 *
 * It uses a private stack with size 64 internally.
 */
class hiprtSceneTraversalAnyHit {
 public:
  HIPRT_DEVICE hiprtSceneTraversalAnyHit(hiprtScene scene,
                                         const hiprtRay &ray,
                                         hiprtRayMask mask = hiprtFullRayMask,
                                         hiprtTraversalHint hint = hiprtTraversalHintDefault,
                                         void *payload = nullptr,
                                         hiprtFuncTable funcTable = nullptr,
                                         u32 rayType = 0,
                                         float time = 0.0f);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtSceneTraversal_impl<hiprtTraversalTerminateAtAnyHit>,
             SIZE_SCENE_TRAVERSAL_PRIVATE_STACK,
             ALIGNMENT_SCENE_TRAVERSAL_PRIVATE_STACK>
      m_impl;
};

/** \brief A traversal object for finding the closest hit with hiprtGeometry containing triangles.
 *
 * \tparam hiprtStack A custom stack.
 */
template<typename hiprtStack,
         hiprtPrimitiveNodeType PrimitiveNodeType,
         hiprtTraversalType TraversalType>
class hiprtGeomTraversalCustomStack_impl;

template<typename hiprtStack> class hiprtGeomTraversalClosestCustomStack {
 public:
  HIPRT_DEVICE hiprtGeomTraversalClosestCustomStack(
      hiprtGeometry geom,
      const hiprtRay &ray,
      hiprtStack &stack,
      hiprtTraversalHint hint = hiprtTraversalHintDefault,
      void *payload = nullptr,
      hiprtFuncTable funcTable = nullptr,
      u32 rayType = 0);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtGeomTraversalCustomStack_impl<hiprtStack,
                                                hiprtTriangleNode,
                                                hiprtTraversalTerminateAtClosestHit>,
             SIZE_GEOM_TRAVERSAL_CUSTOM_STACK,
             ALIGNMENT_GEOM_TRAVERSAL_CUSTOM_STACK>
      m_impl;
};

/** \brief A traversal object for finding the any hit with hiprtGeometry containing triangles.
 *
 * \tparam hiprtStack A custom stack.
 */
template<typename hiprtStack> class hiprtGeomTraversalAnyHitCustomStack {
 public:
  HIPRT_DEVICE hiprtGeomTraversalAnyHitCustomStack(
      hiprtGeometry geom,
      const hiprtRay &ray,
      hiprtStack &stack,
      hiprtTraversalHint hint = hiprtTraversalHintDefault,
      void *payload = nullptr,
      hiprtFuncTable funcTable = nullptr,
      u32 rayType = 0);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtGeomTraversalCustomStack_impl<hiprtStack,
                                                hiprtTriangleNode,
                                                hiprtTraversalTerminateAtAnyHit>,
             SIZE_GEOM_TRAVERSAL_CUSTOM_STACK,
             ALIGNMENT_GEOM_TRAVERSAL_CUSTOM_STACK>
      m_impl;
};

/** \brief A traversal object for finding the closest hit with hiprtGeometry containing custom
 * primitives.
 *
 * \tparam hiprtStack A custom stack.
 */
template<typename hiprtStack> class hiprtGeomCustomTraversalClosestCustomStack {
 public:
  HIPRT_DEVICE hiprtGeomCustomTraversalClosestCustomStack(
      hiprtGeometry geom,
      const hiprtRay &ray,
      hiprtStack &stack,
      hiprtTraversalHint hint = hiprtTraversalHintDefault,
      void *payload = nullptr,
      hiprtFuncTable funcTable = nullptr,
      u32 rayType = 0);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtGeomTraversalCustomStack_impl<hiprtStack,
                                                hiprtCustomNode,
                                                hiprtTraversalTerminateAtClosestHit>,
             SIZE_GEOM_TRAVERSAL_CUSTOM_STACK,
             ALIGNMENT_GEOM_TRAVERSAL_CUSTOM_STACK>
      m_impl;
};

/** \brief A traversal object for finding the any hit with hiprtGeometry containing custom
 * primitives.
 *
 * \tparam hiprtStack A custom stack.
 */
template<typename hiprtStack> class hiprtGeomCustomTraversalAnyHitCustomStack {
 public:
  HIPRT_DEVICE hiprtGeomCustomTraversalAnyHitCustomStack(
      hiprtGeometry geom,
      const hiprtRay &ray,
      hiprtStack &stack,
      hiprtTraversalHint hint = hiprtTraversalHintDefault,
      void *payload = nullptr,
      hiprtFuncTable funcTable = nullptr,
      u32 rayType = 0);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtGeomTraversalCustomStack_impl<hiprtStack,
                                                hiprtCustomNode,
                                                hiprtTraversalTerminateAtAnyHit>,
             SIZE_GEOM_TRAVERSAL_CUSTOM_STACK,
             ALIGNMENT_GEOM_TRAVERSAL_CUSTOM_STACK>
      m_impl;
};

/** \brief A traversal object for finding the closest hit with hiprtScene.
 *
 * \tparam hiprtStack A custom stack.
 */
template<typename hiprtStack, hiprtTraversalType TraversalType>
class hiprtSceneTraversalCustomStack_impl;

template<typename hiprtStack> class hiprtSceneTraversalClosestCustomStack {
 public:
  HIPRT_DEVICE hiprtSceneTraversalClosestCustomStack(
      hiprtScene scene,
      const hiprtRay &ray,
      hiprtStack &stack,
      hiprtRayMask mask = hiprtFullRayMask,
      hiprtTraversalHint hint = hiprtTraversalHintDefault,
      void *payload = nullptr,
      hiprtFuncTable funcTable = nullptr,
      u32 rayType = 0,
      float time = 0.0f);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtSceneTraversalCustomStack_impl<hiprtStack, hiprtTraversalTerminateAtClosestHit>,
             SIZE_SCENE_TRAVERSAL_CUSTOM_STACK,
             ALIGNMENT_SCENE_TRAVERSAL_CUSTOM_STACK>
      m_impl;
};

/** \brief A traversal object for finding the any hit with hiprtScene.
 *
 * \tparam hiprtStack A custom stack.
 */
template<typename hiprtStack> class hiprtSceneTraversalAnyHitCustomStack {
 public:
  HIPRT_DEVICE hiprtSceneTraversalAnyHitCustomStack(
      hiprtScene scene,
      const hiprtRay &ray,
      hiprtStack &stack,
      hiprtRayMask mask = hiprtFullRayMask,
      hiprtTraversalHint hint = hiprtTraversalHintDefault,
      void *payload = nullptr,
      hiprtFuncTable funcTable = nullptr,
      u32 rayType = 0,
      float time = 0.0f);
  HIPRT_DEVICE hiprtHit getNextHit();
  HIPRT_DEVICE hiprtTraversalState getCurrentState();

 private:
  hiprtPimpl<hiprtSceneTraversalCustomStack_impl<hiprtStack, hiprtTraversalTerminateAtAnyHit>,
             SIZE_SCENE_TRAVERSAL_CUSTOM_STACK,
             ALIGNMENT_SCENE_TRAVERSAL_CUSTOM_STACK>
      m_impl;
};
