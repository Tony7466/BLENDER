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
#ifdef _MSC_VER
#  if _MSC_VER < 1900
#    define snprintf _snprintf
#  endif
#  define popen _popen
#  define pclose _pclose
#  define _CRT_SECURE_NO_WARNINGS
#endif

#include <hiprtew.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  define VC_EXTRALEAN
#  include <windows.h>
#endif
static HMODULE hiprt_lib;

thiprtCreateContext *hiprtCreateContext;
thiprtDestroyContext *hiprtDestroyContext;
thiprtCreateGeometry *hiprtCreateGeometry;
thiprtDestroyGeometry *hiprtDestroyGeometry;
thiprtBuildGeometry *hiprtBuildGeometry;
thiprtGetGeometryBuildTemporaryBufferSize *hiprtGetGeometryBuildTemporaryBufferSize;
thiprtCreateScene *hiprtCreateScene;
thiprtDestroyScene *hiprtDestroyScene;
thiprtBuildScene *hiprtBuildScene;
thiprtGetSceneBuildTemporaryBufferSize *hiprtGetSceneBuildTemporaryBufferSize;
thiprtCreateFuncTable *hiprtCreateFuncTable;
thiprtSetFuncTable *hiprtSetFuncTable;
thiprtDestroyFuncTable *hiprtDestroyFuncTable;

static void hipewHipRtExit(void)
{
  if (hiprt_lib != NULL) {
    /*  Ignore errors. */
    FreeLibrary(hiprt_lib);
    hiprt_lib = NULL;
  }
}

bool InitHIPRT()
{
  static bool initialized = false;

  if (initialized) {
    return true;
  }

#  ifdef _WIN32
 

  initialized = true;

  if (atexit(hipewHipRtExit)) {
    return false;
  }

  std::string hiprt_ver(HIPRT_VERSION_STR);
  std::string hiprt_path = "hiprt" + hiprt_ver + "64.dll";

    hiprt_lib = LoadLibraryA(hiprt_path.c_str());

  if (hiprt_lib == NULL){
    return false;
    }

  hiprtCreateContext = (thiprtCreateContext*)GetProcAddress(hiprt_lib, "hiprtCreateContext");
  hiprtDestroyContext = (thiprtDestroyContext*)GetProcAddress(hiprt_lib, "hiprtDestroyContext");
  hiprtCreateGeometry = (thiprtCreateGeometry*)GetProcAddress(hiprt_lib, "hiprtCreateGeometry");
  hiprtDestroyGeometry = (thiprtDestroyGeometry*)GetProcAddress(hiprt_lib, "hiprtDestroyGeometry");
  hiprtBuildGeometry = (thiprtBuildGeometry*)GetProcAddress(hiprt_lib, "hiprtBuildGeometry");
  hiprtGetGeometryBuildTemporaryBufferSize = (thiprtGetGeometryBuildTemporaryBufferSize*)
      GetProcAddress(
      hiprt_lib, "hiprtGetGeometryBuildTemporaryBufferSize");
  hiprtCreateScene = (thiprtCreateScene*)GetProcAddress(hiprt_lib, "hiprtCreateScene");
  hiprtDestroyScene = (thiprtDestroyScene*)GetProcAddress(hiprt_lib, "hiprtDestroyScene");
  hiprtBuildScene = (thiprtBuildScene*)GetProcAddress(hiprt_lib, "hiprtBuildScene");
  hiprtGetSceneBuildTemporaryBufferSize = (thiprtGetSceneBuildTemporaryBufferSize*)GetProcAddress(
      hiprt_lib, "hiprtGetSceneBuildTemporaryBufferSize");
  hiprtCreateFuncTable = (thiprtCreateFuncTable*)GetProcAddress(hiprt_lib, "hiprtCreateFuncTable");
  hiprtSetFuncTable = (thiprtSetFuncTable*)GetProcAddress(hiprt_lib, "hiprtSetFuncTable");
  hiprtDestroyFuncTable = (thiprtDestroyFuncTable*)GetProcAddress(hiprt_lib, "hiprtDestroyFuncTable");

  return true;

#  else

  return false;

#  endif
}
