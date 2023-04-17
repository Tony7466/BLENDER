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