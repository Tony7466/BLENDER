#include <AppKit/AppKit.h>

#import <Quartz/Quartz.h>
#import <QuickLookThumbnailing/QuickLookThumbnailing.h>

NS_ASSUME_NONNULL_BEGIN

@interface ThumbnailProvider : QLThumbnailProvider

@end

@interface PreviewViewController : NSViewController <QLPreviewingController>

@end

NS_ASSUME_NONNULL_END
