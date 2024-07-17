#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2015-2023 Blender Authors
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import platform
import os
import shlex
import sys
from pathlib import Path

# List of .blend files that are known to be failing and are not ready to be
# tested, or that only make sense on some devices. Accepts regular expressions.
BLACKLIST_ALL = [
    # Blacklisted due overlapping object differences between platforms.
    "hair_geom_reflection.blend",
    "hair_geom_transmission.blend",
    "hair_instancer_uv.blend",
    "principled_hair_directcoloring.blend",
    "visibility_particles.blend",
]

BLOCKLIST_EXPLICIT_OSL_TESTS = [
    # OSL only supported on CPU.
    '.*_osl.blend',
    'osl_.*.blend',
]

BLOCKLIST_OSL = [
    # Block tests that fail with OSL due to differences from SVM.
    # Point Density is disabled on surfaces in SVM.
    # This was a performance optimization that wasn't carried over to OSL.
    'T49936.blend',
    'T49936_indirect.blend',
    # AOVs are not supported. See 73266
    'aov_position.blend',
    'render_passes_aov.*.blend',
    # OSL uses mipmapping when loading from disk?
    'image_byte.*.blend',
    'image_float.*.blend',
    'image_half.*.blend',
    'image_mapping_.*_closest.blend',
    'image_mapping_.*_cubic.blend',
    'image_mapping_.*_linear.blend',
    # OSL handles bump + displacement differently from SVM
    'both_displacement.blend',
    'bump_with_displacement.blend',
    'ray_portal.blend',
    # TODO: Add more failing tests
    # Currently failing tests that aren't in this list are:
    # Some image color space tests - Likely due to differences in how textures are processed
    # Blackbody - Subtle noise difference
    # Various Principled BSDF tests - Seems to be noise from mixing and layering
    # Some render pass tests - Seems to be noise
]

BLACKLIST_OPTIX = [
    # Ray intersection precision issues
    'T50164.blend',
    'T43865.blend',
]

BLOCKLIST_OPTIX_OSL = [
    # OPTIX OSL doesn't support trace function needed for AO and bevel
    'bake_bevel.blend',
    'ambient_occlusion.*.blend',
    'bevel.blend',
    'osl_trace_shader.blend',
    # The Volumetric noise texture is different for some reason
    'principled_absorption.blend',
    # Dicing tests use wireframe node which doesn't appear to be supported in OptiX
    'dicing_camera.blend',
    'offscreen_dicing.blend',
    'panorama_dicing.blend',
    # Bump evaluation is not implemented yet. See 104276
    'compare_bump.blend',
    'both_displacement.blend',
    'bump_with_displacement.blend',
    'ray_portal.blend',
    # TODO: Investigate every other failing case and add them here.
    # Currently failing tests that aren't in this list are:
    # ray_portal*.blend - CUDA error
    # image_mapping_udim*.blend - Can't load UDIM from disk? But can load UDIM if it's packed, but doesn't seem to use it properly.
    # points_volume.blend - CUDA error
    # principled_emission_alpha.blend - CUDA error related to connected inputs. Probably the same as 122779
    # point_density_*_object - Object scale doesn't appear to be appplied to texture
    # All the other tests mentioned in BLOCKLIST_OSL (E.g. Principled BSDF tests having noise differences)
]

BLACKLIST_METAL = []

if platform.system() == "Darwin":
    version, _, _ = platform.mac_ver()
    major_version = version.split(".")[0]
    if int(major_version) < 13:
        BLACKLIST_METAL += [
            # MNEE only works on Metal with macOS >= 13
            "underwater_caustics.blend",
        ]

BLACKLIST_GPU = [
    # Uninvestigated differences with GPU.
    'image_log.blend',
    'T40964.blend',
    'T45609.blend',
    'smoke_color.blend',
    'bevel_mblur.blend',
    # Inconsistency between Embree and Hair primitive on GPU.
    'denoise_hair.blend',
    'hair_basemesh_intercept.blend',
    'hair_instancer_uv.blend',
    'hair_length_info.blend',
    'hair_particle_random.blend',
    "hair_transmission.blend",
    'principled_hair_.*.blend',
    'transparent_shadow_hair.*.blend',
    "microfacet_hair_orientation.blend",
    # Inconsistent handling of overlapping objects.
    "T41143.blend",
    "visibility_particles.blend",
    # No path guiding on GPU.
    "guiding*.blend",
]


def get_arguments(filepath, output_filepath, osl=False):
    dirname = os.path.dirname(filepath)
    basedir = os.path.dirname(dirname)
    subject = os.path.basename(dirname)

    args = [
        "--background",
        "--factory-startup",
        "--enable-autoexec",
        "--debug-memory",
        "--debug-exit-on-error",
        filepath,
        "-E", "CYCLES",
        "-o", output_filepath,
        "-F", "PNG"]

    # OSL and GPU examples
    # custom_args += ["--python-expr", "import bpy; bpy.context.scene.cycles.shading_system = True"]
    # custom_args += ["--python-expr", "import bpy; bpy.context.scene.cycles.device = 'GPU'"]
    custom_args = os.getenv('CYCLESTEST_ARGS')
    if custom_args:
        args.extend(shlex.split(custom_args))

    spp_multiplier = os.getenv('CYCLESTEST_SPP_MULTIPLIER')
    if spp_multiplier:
        args.extend(["--python-expr", f"import bpy; bpy.context.scene.cycles.samples *= {spp_multiplier}"])

    if osl:
        args.extend(["--python-expr", "import bpy; bpy.context.scene.cycles.shading_system = True"])

    if subject == 'bake':
        args.extend(['--python', os.path.join(basedir, "util", "render_bake.py")])
    elif subject == 'denoise_animation':
        args.extend(['--python', os.path.join(basedir, "util", "render_denoise.py")])
    else:
        args.extend(["-f", "1"])

    return args


def create_argparse():
    parser = argparse.ArgumentParser()
    parser.add_argument("-blender", nargs="+")
    parser.add_argument("-testdir", nargs=1)
    parser.add_argument("-outdir", nargs=1)
    parser.add_argument("-oiiotool", nargs=1)
    parser.add_argument("-device", nargs=1)
    parser.add_argument("-blacklist", nargs="*")
    parser.add_argument("-osl", default=False, action='store_true')
    parser.add_argument('--batch', default=False, action='store_true')
    return parser


def main():
    parser = create_argparse()
    args = parser.parse_args()

    blender = args.blender[0]
    test_dir = args.testdir[0]
    oiiotool = args.oiiotool[0]
    output_dir = args.outdir[0]
    device = args.device[0]

    blacklist = BLACKLIST_ALL
    if device != 'CPU':
        blacklist += BLACKLIST_GPU
    if device != 'CPU' or 'OSL' in args.blacklist:
        blacklist += BLACKLIST_EXPLICIT_OSL_TESTS
    if device == 'OPTIX':
        blacklist += BLACKLIST_OPTIX
        if args.osl:
            blacklist += BLOCKLIST_OPTIX_OSL
    if device == 'METAL':
        blacklist += BLACKLIST_METAL
    if args.osl:
        blacklist += BLOCKLIST_OSL

    from modules import render_report
    report = render_report.Report('Cycles', output_dir, oiiotool, device, blacklist, args.osl)
    report.set_pixelated(True)
    report.set_reference_dir("cycles_renders")
    if device == 'CPU':
        report.set_compare_engine('eevee')
    else:
        report.set_compare_engine('cycles', 'CPU')

    # Increase threshold for motion blur, see #78777.
    #
    # underwater_caustics.blend gives quite different results on Linux and Intel macOS compared to
    # Windows and Arm macOS.
    test_dir_name = Path(test_dir).name
    if test_dir_name in {'motion_blur', 'integrator'}:
        report.set_fail_threshold(0.032)

    ok = report.run(test_dir, blender, get_arguments, batch=args.batch)

    sys.exit(not ok)


if __name__ == "__main__":
    main()
