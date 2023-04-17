lsregister and its listing and detection

more logs in lldb

more logs in console: look for anything qlmanage

must sign the appex bundle with sandbox

must launch the app

appex cannot write at all : inferred from

error	17:53:57.863578+0530	kernel	Sandbox: qlmanage(37213) deny(1) file-write-data /dev/dtracehelper
error	17:53:58.033957+0530	kernel	Sandbox: qlmanage(37213) deny(1) mach-register com.apple.axserver (per-pid)
error	17:53:58.071424+0530	kernel	Sandbox: qlmanage(37213) deny(1) mach-register com.apple.tsm.portname (per-pid)
error	17:53:58.164827+0530	kernel	Sandbox: qlmanage(37213) deny(1) user-preference-read com.apple.HIToolbox
error	17:54:00.837220+0530	kernel	Sandbox: qlmanage(37213) deny(1) file-write-data /dev/dtracehelper
error	17:54:00.904484+0530	kernel	Sandbox: qlmanage(37213) deny(1) file-write-create /Users/ankitkumar/Library/Saved Application State/com.apple.quicklook.qlmanage.savedState

https://eclecticlight.co/2018/04/05/inside-quicklook-previews-with-qlmanage/

in info.plist, add NSExtensionPrincipalClass, add QLThumbnailMinimumDimension

Try removing blend file thumbnail specificaion from blender.app info.plist

Expect

lender-thumbnailer	__extensionPrincipalClass != nil - /AppleInternal/Library/BuildRoots/a0876c02-1788-11ed-b9c4-96898e02b808/Library/Caches/com.apple.xbs/Sources/ExtensionFoundation/ExtensionFoundation/Source/NSExtension/NSExtensionSupport/EXConcreteExtensionContextVendor.m:108: Unable to find NSExtensionPrincipalClass (<private>) in extension bundle! Please verify that the extension links the required frameworks and that the value for NSExtensionPrincipalClass is prefixed with '$(PRODUCT_MODULE_NAME).' if the class is implemented in Swift.
fault	00:55:16.449383+0530	blender-thumbnailer	[__extensionPrincipalClass conformsToProtocol:@protocol(NSExtensionRequestHandling)] - /AppleInternal/Library/BuildRoots/a0876c02-1788-11ed-b9c4-96898e02b808/Library/Caches/com.apple.xbs/Sources/ExtensionFoundation/ExtensionFoundation/Source/NSExtension/NSExtensionSupport/EXConcreteExtensionContextVendor.m:109: NSExtensionPrincipalClass does not conform to NSExtensionRequestHandling protocol!


Signing blender.app

move blender.app to /Applications or launch it at least once

Don't access files outside sandbox or kernel denies:

default	03:07:03.525360+0530	blender-thumbnailer	hello world from blender
error	03:07:03.527837+0530	blender-thumbnailer	Couldn't issue file extension for url: <private>
error	03:07:03.531152+0530	kernel	Sandbox: com.apple.quickl(476) deny(1) file-read-data /Users/ankitkumar/Pictures/IMG_3158.JPG

restart ?
