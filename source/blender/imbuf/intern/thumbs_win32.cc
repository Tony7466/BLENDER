/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup imbuf
 */

#include <shlobj_core.h>
#include <commoncontrols.h>
#include <thumbcache.h>
#include <wchar.h>

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"
#include "IMB_thumbs_win32.hh"

#include "BLI_utildefines.h"
#include "BLI_path_util.h"

#include "MEM_guardedalloc.h"

#include "utfconv.hh"

/* Fancy preview folder icons. stl, fbx, glb, obj previews. */
#define IMB_THUMB_WIN32_PREVIEW true

/* EXE images, generic folder icons. */
#define IMB_THUMB_WIN32_ICON true

HBITMAP imb_thumb_win32_preview(const PCWSTR path, const size_t size)
{
  IShellItem *item = nullptr;
  HRESULT hr = SHCreateItemFromParsingName(path, nullptr, IID_PPV_ARGS(&item));
  if (FAILED(hr)) {
    return nullptr;
  }

  IThumbnailCache *cache = nullptr;
  hr = CoCreateInstance(CLSID_LocalThumbnailCache, nullptr, CLSCTX_INPROC, IID_PPV_ARGS(&cache));
  if (FAILED(hr)) {
    return nullptr;
  }

  ISharedBitmap *shared_bitmap = nullptr;
  WTS_THUMBNAILID thumbnailId;
  hr = cache->GetThumbnail(item, size, WTS_FASTEXTRACT, &shared_bitmap, nullptr, &thumbnailId);
  if (FAILED(hr)) {
    return nullptr;
  }

  HBITMAP bitmap = nullptr;
  hr = shared_bitmap->GetSharedBitmap(&bitmap);
  if (FAILED(hr)) {
    return nullptr;
  }
  return bitmap;
}

HBITMAP imb_thumb_win32_icon(const PCWSTR path, const size_t size)
{
  IShellItemImageFactory *pImageFactory = nullptr;
  HRESULT hr = ::SHCreateItemFromParsingName(path, nullptr, IID_PPV_ARGS(&pImageFactory));
  if (FAILED(hr)) {
    return nullptr;
  }

  HBITMAP bitmap = nullptr;
  SIZE size_v2 = {(LONG)size, (LONG)size};
  hr = pImageFactory->GetImage(size_v2, SIIGBF_RESIZETOFIT | SIIGBF_ICONONLY, &bitmap);
  pImageFactory->Release();
  if (FAILED(hr)) {
    return nullptr;
  }
  return bitmap;
}

struct ImBuf *IMB_thumb_win32(const char *file_path, const size_t size)
{
  if (size < 1) {
    return nullptr;
  }

  WCHAR path_utf16[FILE_MAXDIR] = {0};
  if (conv_utf_8_to_16(file_path, path_utf16, ARRAY_SIZE(path_utf16)) != 0) {
    return nullptr;
  }

  HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
  if (FAILED(hr)) {
    return nullptr;
  }

  HBITMAP thumbnail = nullptr;
  bool is_preview = false;

  bool is_icon = false;

  if (IMB_THUMB_WIN32_PREVIEW) {
    thumbnail = imb_thumb_win32_preview(path_utf16, size);
    is_preview = (thumbnail != nullptr);
  }

  if (IMB_THUMB_WIN32_ICON && thumbnail == nullptr) {
    thumbnail = imb_thumb_win32_icon(path_utf16, size);
    is_icon = (thumbnail != nullptr);
  }

  if (thumbnail == nullptr) {
    CoUninitialize();
    return nullptr;
  }

  void *buff = nullptr;
  BITMAP bmp;
  int w, h, channels;

  if (::GetObject(thumbnail, sizeof(bmp), (LPVOID)&bmp)) {
    w = bmp.bmWidth;
    h = bmp.bmHeight;
    channels = (bmp.bmPlanes * bmp.bmBitsPixel + 7) >> 3;
    size_t buff_size = bmp.bmWidthBytes * bmp.bmHeight;
    buff = (uchar *)MEM_mallocN(buff_size, __func__);
    uchar *source = (uchar *)bmp.bmBits;
    uchar *dest = (uchar *)buff;
    for (int i = 0; i < buff_size; i += 4) {
      dest[i] = source[i + 2];
      dest[i + 1] = source[i + 1];
      dest[i + 2] = source[i];
      dest[i + 3] = source[i + 3];
    }
  }

  DeleteObject(thumbnail);
  CoUninitialize();

  if (buff) {
    ImBuf *img = IMB_allocImBuf(w, h, 32, 0);
    if (img) {
      img->channels = channels;
      img->byte_buffer.data = (uint8_t *)buff;
      img->flags |= IB_rect;
      // img->mall |= IB_rect;
      if (is_preview) {
        IMB_flipy(img);
      }
      return img;
    }
  }

  return nullptr;
}

#undef IMB_THUMB_WIN32_PREVIEW
#undef IMB_THUMB_WIN32_ICON
