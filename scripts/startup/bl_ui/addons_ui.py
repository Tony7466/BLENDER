# SPDX-FileCopyrightText: 2009-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.app.translations import (
    pgettext_iface as iface_,
)

class AddonsUI:
    _support_icon_mapping = {
        'OFFICIAL': 'BLENDER',
        'COMMUNITY': 'COMMUNITY',
        'TESTING': 'EXPERIMENTAL',
    }

    @staticmethod
    def is_user_addon(mod, user_addon_paths):
        import os

        if not user_addon_paths:
            for path in (
                    bpy.utils.script_path_user(),
                    *bpy.utils.script_paths_pref(),
            ):
                if path is not None:
                    user_addon_paths.append(os.path.join(path, "addons"))

        for path in user_addon_paths:
            if bpy.path.is_subdir(mod.__file__, path):
                return True
        return False


    @staticmethod
    def draw_error(layout, message):
        lines = message.split("\n")
        box = layout.box()
        sub = box.row()
        sub.label(text=lines[0])
        sub.label(icon='ERROR')
        for l in lines[1:]:
            box.label(text=l)


    @staticmethod
    def draw(context, layout):
        import os
        import addon_utils

        wm = context.window_manager
        prefs = context.preferences
        used_ext = {ext.module for ext in prefs.addons}

        addon_user_dirs = tuple(
            p for p in (
                *[os.path.join(pref_p, "addons") for pref_p in bpy.utils.script_paths_pref()],
                bpy.utils.user_resource('SCRIPTS', path="addons"),
            )
            if p
        )

        # collect the categories that can be filtered on
        addons = [
            (mod, addon_utils.module_bl_info(mod))
            for mod in addon_utils.modules(refresh=False)
        ]

        split = layout.split(factor=0.6)

        row = split.row()
        row.prop(wm, "addon_support", expand=True)

        row = split.row(align=True)
        row.operator("preferences.addon_install", icon='IMPORT', text="Install...")
        row.operator("preferences.addon_refresh", icon='FILE_REFRESH', text="Refresh")

        row = layout.row()
        row.prop(prefs.view, "show_addons_enabled_only")
        row.prop(wm, "addon_filter", text="")
        row.prop(wm, "addon_search", text="", icon='VIEWZOOM')

        col = layout.column()

        # set in addon_utils.modules_refresh()
        if addon_utils.error_duplicates:
            box = col.box()
            row = box.row()
            row.label(text="Multiple add-ons with the same name found!")
            row.label(icon='ERROR')
            box.label(text="Delete one of each pair to resolve:")
            for (addon_name, addon_file, addon_path) in addon_utils.error_duplicates:
                box.separator()
                sub_col = box.column(align=True)
                sub_col.label(text=addon_name + ":")
                sub_col.label(text="    " + addon_file)
                sub_col.label(text="    " + addon_path)

        if addon_utils.error_encoding:
            AddonsUI.draw_error(
                col,
                "One or more addons do not have UTF-8 encoding\n"
                "(see console for details)",
            )

        show_enabled_only = prefs.view.show_addons_enabled_only
        filter = wm.addon_filter
        search = wm.addon_search.lower()
        support = wm.addon_support

        # initialized on demand
        user_addon_paths = []

        for mod, info in addons:
            module_name = mod.__name__

            is_enabled = module_name in used_ext

            if info["support"] not in support:
                continue

            # check if addon should be visible with current filters
            is_visible = (
                (filter == "All") or
                (filter == info["category"]) or
                (filter == "User" and (mod.__file__.startswith(addon_user_dirs)))
            )
            if show_enabled_only:
                is_visible = is_visible and is_enabled

            if is_visible:
                if search and not (
                        (search in info["name"].lower() or
                         search in iface_(info["name"]).lower()) or
                        (info["author"] and (search in info["author"].lower())) or
                        ((filter == "All") and (search in info["category"].lower() or
                                                search in iface_(info["category"]).lower()))
                ):
                    continue

                # Addon UI Code
                col_box = col.column()
                box = col_box.box()
                colsub = box.column()
                row = colsub.row(align=True)

                row.operator(
                    "preferences.addon_expand",
                    icon='DISCLOSURE_TRI_DOWN' if info["show_expanded"] else 'DISCLOSURE_TRI_RIGHT',
                    emboss=False,
                ).module = module_name

                row.operator(
                    "preferences.addon_disable" if is_enabled else "preferences.addon_enable",
                    icon='CHECKBOX_HLT' if is_enabled else 'CHECKBOX_DEHLT', text="",
                    emboss=False,
                ).module = module_name

                sub = row.row()
                sub.active = is_enabled
                sub.label(text=iface_("%s: %s") % (iface_(info["category"]), iface_(info["name"])))

                if info["warning"]:
                    sub.label(icon='ERROR')

                # icon showing support level.
                sub.label(icon=AddonsUI._support_icon_mapping.get(info["support"], 'QUESTION'))

                # Expanded UI (only if additional info is available)
                if info["show_expanded"]:
                    if info["description"]:
                        split = colsub.row().split(factor=0.15)
                        split.label(text="Description:")
                        split.label(text=tip_(info["description"]))
                    if info["location"]:
                        split = colsub.row().split(factor=0.15)
                        split.label(text="Location:")
                        split.label(text=tip_(info["location"]))
                    if mod:
                        split = colsub.row().split(factor=0.15)
                        split.label(text="File:")
                        split.label(text=mod.__file__, translate=False)
                    if info["author"]:
                        split = colsub.row().split(factor=0.15)
                        split.label(text="Author:")
                        split.label(text=info["author"], translate=False)
                    if info["version"]:
                        split = colsub.row().split(factor=0.15)
                        split.label(text="Version:")
                        split.label(text=".".join(str(x) for x in info["version"]), translate=False)
                    if info["warning"]:
                        split = colsub.row().split(factor=0.15)
                        split.label(text="Warning:")
                        split.label(text="  " + info["warning"], icon='ERROR')

                    user_addon = USERPREF_PT_addons.is_user_addon(mod, user_addon_paths)
                    if info["doc_url"] or info.get("tracker_url"):
                        split = colsub.row().split(factor=0.15)
                        split.label(text="Internet:")
                        sub = split.row()
                        if info["doc_url"]:
                            sub.operator(
                                "wm.url_open", text="Documentation", icon='HELP',
                            ).url = info["doc_url"]
                        # Only add "Report a Bug" button if tracker_url is set
                        # or the add-on is bundled (use official tracker then).
                        if info.get("tracker_url"):
                            sub.operator(
                                "wm.url_open", text="Report a Bug", icon='URL',
                            ).url = info["tracker_url"]
                        elif not user_addon:
                            addon_info = (
                                "Name: %s %s\n"
                                "Author: %s\n"
                            ) % (info["name"], str(info["version"]), info["author"])
                            props = sub.operator(
                                "wm.url_open_preset", text="Report a Bug", icon='URL',
                            )
                            props.type = 'BUG_ADDON'
                            props.id = addon_info

                    if user_addon:
                        split = colsub.row().split(factor=0.15)
                        split.label(text="User:")
                        split.operator(
                            "preferences.addon_remove", text="Remove", icon='CANCEL',
                        ).module = mod.__name__

                    # Show addon user preferences
                    if is_enabled:
                        addon_preferences = prefs.addons[module_name].preferences
                        if addon_preferences is not None:
                            draw = getattr(addon_preferences, "draw", None)
                            if draw is not None:
                                addon_preferences_class = type(addon_preferences)
                                box_prefs = col_box.box()
                                box_prefs.label(text="Preferences:")
                                addon_preferences_class.layout = box_prefs
                                try:
                                    draw(context)
                                except BaseException:
                                    import traceback
                                    traceback.print_exc()
                                    box_prefs.label(text="Error (see console)", icon='ERROR')
                                del addon_preferences_class.layout

        # Append missing scripts
        # First collect scripts that are used but have no script file.
        module_names = {mod.__name__ for mod, info in addons}
        missing_modules = {ext for ext in used_ext if ext not in module_names}

        if missing_modules and filter in {"All", "Enabled"}:
            col.column().separator()
            col.column().label(text="Missing script files")

            module_names = {mod.__name__ for mod, info in addons}
            for module_name in sorted(missing_modules):
                is_enabled = module_name in used_ext
                # Addon UI Code
                box = col.column().box()
                colsub = box.column()
                row = colsub.row(align=True)

                row.label(text="", icon='ERROR')

                if is_enabled:
                    row.operator(
                        "preferences.addon_disable", icon='CHECKBOX_HLT', text="", emboss=False,
                    ).module = module_name

                row.label(text=module_name, translate=False)
