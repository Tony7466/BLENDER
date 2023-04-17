#include <cstdio>
#include <fstream>
#include <unistd.h>

#import <Foundation/Foundation.h>
#import <QuickLook/QLBase.h>
#import <QuickLook/QuickLook.h>

#import "ThumbnailProvider.hh"

#include "BLI_fileops.h"
#include "BLI_filereader.h"
#include "blendthumb.hh"

class FileReaderRAII {
  int src_file_ = -1;

 public:
  explicit FileReaderRAII(int src_file) : src_file_(src_file) {}
  ~FileReaderRAII()
  {
    if (src_file_ != -1) {
      close(src_file_);
    }
  }

  bool good()
  {
    return src_file_ != -1;
  }

  int get()
  {
    return src_file_;
  }
};

static eThumbStatus creator_impl(const char *src_blend_path)
{
  eThumbStatus err;

  /* Open source file `src_blend`. */
  FileReaderRAII src_file_fd = FileReaderRAII(BLI_open(src_blend_path, O_BINARY | O_RDONLY, 0));
  if (!src_file_fd.good()) {
    return BT_FILE_ERR;
  }

  /* Thumbnail reading is responsible for freeing `file` and closing `src_file`. */
  FileReader *file_content = BLI_filereader_new_file(src_file_fd.get());
  if (file_content == nullptr) {
    return BT_FILE_ERR;
  }

  /* Extract thumbnail from file. */
  Thumbnail thumb;
  err = blendthumb_create_thumb_from_file(file_content, &thumb);
  if (err != BT_OK) {
    return err;
  }

  return err;
}

@implementation ThumbnailProvider

- (void)provideThumbnailForFileRequest:(QLFileThumbnailRequest *)request
                     completionHandler:(void (^)(QLThumbnailReply *_Nullable reply,
                                                 NSError *_Nullable error))handler
{
  @autoreleasepool {  // Add the supported content types to the QLSupportedContentTypes array in
                      // the Info.plist of the
    // extension.

    NSLog(@"hello world from blender");
    NSURL *foo = [[NSURL alloc]
        initFileURLWithFileSystemRepresentation:"/Users/ankitkumar/Pictures/IMG_3158.JPG"
                                    isDirectory:NO
                                  relativeToURL:nil];
    QLThumbnailReply *reply = [QLThumbnailReply replyWithImageFileURL:foo];

    handler(reply, nil);
  }
}

@end
