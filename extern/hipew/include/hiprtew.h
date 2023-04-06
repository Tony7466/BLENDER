/*
 * Copyright 2011-2021 Blender Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License
 */

#ifndef __HIPRTEW_H__
#define __HIPRTEW_H__


#include "hiprt/hiprt_types.h"
#ifdef __cplusplus

#endif

#define HIPRT_MAJOR_VERSION 2
#define HIPRT_MINOR_VERSION 0
#define HIPRT_PATCH_VERSION 0xb68861

#define HIPRT_API_VERSION 2000
#define HIPRT_VERSION_STR "02000"

typedef unsigned int       uint32_t;

  #if defined( _MSC_VER )
#ifdef HIPRT_EXPORTS
#define HIPRT_API __declspec( dllexport )
#else
#define HIPRT_API __declspec( dllimport )
#endif
#elif defined( __GNUC__ )
#ifdef HIPRT_EXPORTS
#define HIPRT_API __attribute__( ( visibility( "default" ) ) )
#else
#define HIPRT_API
#endif
#else
#define HIPRT_API
#pragma warning Unknown dynamic link import / export semantics.
#endif

bool InitHIPRT();

typedef HIPRT_API hiprtError(thiprtCreateContext)(uint32_t hiprtApiVersion,
                                                  hiprtContextCreationInput &input,
                                                  hiprtContext *outContext);
typedef HIPRT_API hiprtError(thiprtDestroyContext)(hiprtContext context);
typedef HIPRT_API hiprtError(thiprtCreateGeometry)(hiprtContext context,
                                                   const hiprtGeometryBuildInput *buildInput,
                                                   const hiprtBuildOptions *buildOptions,
                                                   hiprtGeometry *outGeometry);
typedef HIPRT_API hiprtError(thiprtDestroyGeometry)(hiprtContext context,
                                                    hiprtGeometry outGeometry);
typedef HIPRT_API hiprtError(thiprtBuildGeometry)(hiprtContext context,
                                                  hiprtBuildOperation buildOperation,
                                                  const hiprtGeometryBuildInput *buildInput,
                                                  const hiprtBuildOptions *buildOptions,
                                                  hiprtDevicePtr temporaryBuffer,
                                                  hiprtApiStream stream,
                                                  hiprtGeometry outGeometry);
typedef HIPRT_API hiprtError(thiprtGetGeometryBuildTemporaryBufferSize)(
    hiprtContext context,
    const hiprtGeometryBuildInput *buildInput,
    const hiprtBuildOptions *buildOptions,
    size_t *outSize);
typedef HIPRT_API hiprtError(thiprtCreateScene)(hiprtContext context,
                                                const hiprtSceneBuildInput *buildInput,
                                                const hiprtBuildOptions *buildOptions,
                                                hiprtScene *outScene);
typedef HIPRT_API hiprtError(thiprtDestroyScene)(hiprtContext context, hiprtScene outScene);
typedef HIPRT_API hiprtError(thiprtBuildScene)(hiprtContext context,
                                               hiprtBuildOperation buildOperation,
                                               const hiprtSceneBuildInput *buildInput,
                                               const hiprtBuildOptions *buildOptions,
                                               hiprtDevicePtr temporaryBuffer,
                                               hiprtApiStream stream,
                                               hiprtScene outScene);
typedef HIPRT_API hiprtError(thiprtGetSceneBuildTemporaryBufferSize)(
    hiprtContext context,
    const hiprtSceneBuildInput *buildInput,
    const hiprtBuildOptions *buildOptions,
    size_t *outSize);
typedef HIPRT_API hiprtError(thiprtCreateFuncTable)(hiprtContext context,
                                                    uint32_t numGeomTypes,
                                                    uint32_t numRayTypes,
                                                    hiprtFuncTable *outFuncTable);
typedef HIPRT_API hiprtError(thiprtSetFuncTable)(hiprtContext context,
                                                 hiprtFuncTable funcTable,
                                                 uint32_t geomType,
                                                 uint32_t rayType,
                                                 hiprtFuncDataSet set);
typedef HIPRT_API hiprtError(thiprtDestroyFuncTable)(hiprtContext context,
                                                     hiprtFuncTable funcTable);

extern thiprtCreateContext *hiprtCreateContext;
extern thiprtDestroyContext *hiprtDestroyContext;
extern thiprtCreateGeometry *hiprtCreateGeometry;
extern thiprtDestroyGeometry *hiprtDestroyGeometry;
extern thiprtBuildGeometry *hiprtBuildGeometry;
extern thiprtGetGeometryBuildTemporaryBufferSize *hiprtGetGeometryBuildTemporaryBufferSize;
extern thiprtCreateScene *hiprtCreateScene;
extern thiprtDestroyScene *hiprtDestroyScene;
extern thiprtBuildScene *hiprtBuildScene;
extern thiprtGetSceneBuildTemporaryBufferSize *hiprtGetSceneBuildTemporaryBufferSize;
extern thiprtCreateFuncTable *hiprtCreateFuncTable;
extern thiprtSetFuncTable *hiprtSetFuncTable;
extern thiprtDestroyFuncTable *hiprtDestroyFuncTable;



#ifdef __cplusplus
//}
#endif

#endif  /* __HIPRTEW_H__ */
