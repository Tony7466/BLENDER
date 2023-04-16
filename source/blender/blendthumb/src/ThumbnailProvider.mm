#include <cstdio>
#include <fstream>
#include <unistd.h>

#import <Foundation/Foundation.h>
#import <QuickLook/QLBase.h>
#import <QuickLook/QuickLook.h>

#import "ThumbnailProvider.h"

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

@implementation PreviewViewController

- (NSString *)nibName
{
  return @"PreviewViewController";
}

- (void)loadView
{
  [super loadView];
  // Do any additional setup after loading the view.
}

/*
 * Implement this method and set QLSupportsSearchableItems to YES in the Info.plist of the
extension if you support CoreSpotlight.
 *
- (void)preparePreviewOfSearchableItemWithIdentifier:(NSString *)identifier queryString:(NSString
*)queryString completionHandler:(void (^)(NSError * _Nullable))handler {

    // Perform any setup necessary in order to prepare the view.

    // Call the completion handler so Quick Look knows that the preview is fully loaded.
    // Quick Look will display a loading spinner while the completion handler is not called.

    handler(nil);
}
*/

- (void)preparePreviewOfFileAtURL:(NSURL *)url
                completionHandler:(void (^)(NSError *_Nullable))handler
{

  // Add the supported content types to the QLSupportedContentTypes array in the Info.plist of the
  // extension.

  // Perform any setup necessary in order to prepare the view.

  // Call the completion handler so Quick Look knows that the preview is fully loaded.
  // Quick Look will display a loading spinner while the completion handler is not called.

  NSLog(@"blender preparePreviewOfFileAtURL: %@", url);
  std::ofstream file("/Users/ankit.kumar/apps/build_xcode/foobar.txt",
                     std::ios::out | std::ios::trunc | std::ios::binary);
  file << "Hello World";
  file.close();
  std::cout << "Hello World" << std::endl;
  // set image to file path /Users/ankit.kumar/Desktop/Screenshot 2021-09-02 at 1.59.47 PM.png
  [[[self view] imageRepresentation]
      setFileURL:[NSURL fileURLWithPath:@"file:///Users/ankit.kumar/Desktop/Screenshot.png"
                            isDirectory:NO]];
  [[self view] display];
  handler(nil);
}

@end
