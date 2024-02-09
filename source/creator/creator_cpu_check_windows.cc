/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup creator
 */
#include <string>
#include <Windows.h>
#include "BLI_system.h"

extern "C" __declspec(dllexport) void cpu_check_win()
{
  // we need atleast one function to export for the linker to 
  // actually link this library. 
}


BOOL WINAPI DllMain(HINSTANCE /* hinstDLL */,
    DWORD fdwReason, LPVOID /* lpvReserved */)
{
    switch( fdwReason )
    {
        case DLL_PROCESS_ATTACH:
        //if (!BLI_cpu_support_sse41())
        {
          std::string error_title = "Unsupported CPU - " + std::string(BLI_cpu_brand_string());
          MessageBoxA(NULL,
                      "Blender requires a CPU with SSE42 support.",
                      error_title.c_str(),
                      MB_OK | MB_ICONERROR);
          exit(-1);
        }
        break;
   }
    return TRUE;
}

