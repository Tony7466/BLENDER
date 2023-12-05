# SPDX-FileCopyrightText: 2009-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Header, Menu, Panel
from bpy.app.translations import (
    contexts as i18n_contexts,
    pgettext_iface as iface_,
)


class TEXT_HT_header(Header):
    bl_space_type = 'TEXT_EDITOR'

    def draw(self, context):
        layout = self.layout

        st = context.space_data
        text = st.text
        is_syntax_highlight_supported = st.is_syntax_highlight_supported()
        layout.template_header()

        TEXT_MT_editor_menus.draw_collapsible(context, layout)

        layout.separator_spacer()

        if text and text.is_modified:
            row = layout.row(align=True)
            row.alert = True
            row.operator("text.resolve_conflict", text="", icon='QUESTION')

        row = layout.row(align=True)
        row.template_ID(st, "text", new="text.new",
                        unlink="text.unlink", open="text.open")

        if text:
            is_osl = text.name.endswith((".osl", ".osl"))
            if is_osl:
                row.operator("node.shader_script_update",
                             text="", icon='FILE_REFRESH')
            else:
                row = layout.row()
                row.active = is_syntax_highlight_supported
                row.operator("text.run_script", text="", icon='PLAY')

        layout.separator_spacer()

        row = layout.row(align=True)
        row.prop(st, "show_line_numbers", text="")
        row.prop(st, "show_word_wrap", text="")

        syntax = row.row(align=True)
        syntax.active = is_syntax_highlight_supported
        syntax.prop(st, "show_syntax_highlight", text="")


class TEXT_HT_footer(Header):
    bl_space_type = 'TEXT_EDITOR'
    bl_region_type = 'FOOTER'

    def draw(self, context):
        layout = self.layout

        st = context.space_data
        text = st.text
        if text:
            row = layout.row()
            if text.filepath:
                if text.is_dirty:
                    row.label(
                        text=iface_("File: *%s (unsaved)") % text.filepath,
                        translate=False,
                    )
                else:
                    row.label(
                        text=iface_("File: %s") % text.filepath,
                        translate=False,
                    )
            else:
                row.label(
                    text=iface_("Text: External")
                    if text.library
                    else iface_("Text: Internal"),
                    translate=False
                )


class TEXT_MT_editor_menus(Menu):
    bl_idname = "TEXT_MT_editor_menus"
    bl_label = ""

    def draw(self, context):
        layout = self.layout
        st = context.space_data
        text = st.text

        layout.menu("TEXT_MT_view")
        layout.menu("TEXT_MT_text")

        if text:
            layout.menu("TEXT_MT_edit")
            layout.menu("TEXT_MT_select")
            layout.menu("TEXT_MT_format")

        layout.menu("TEXT_MT_templates")


class TEXT_PT_properties(Panel):
    bl_space_type = 'TEXT_EDITOR'
    bl_region_type = 'UI'
    bl_category = "Text"
    bl_label = "Properties"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False
        st = context.space_data

        if not st.text:
            layout.active = False

        st = context.space_data

        col = layout.column(align=False, heading="Margin")
        row = col.row(align=True)
        sub = row.row(align=True)
        sub.prop(st, "show_margin", text="")
        sub = sub.row(align=True)
        sub.active = st.show_margin
        sub.prop(st, "margin_column", text="")

        layout.prop(st, "font_size")
        layout.prop(st, "tab_width")

        text = st.text
        if text:
            layout.prop(text, "indentation")


class TEX_UL_texts_search(bpy.types.UIList):
    @staticmethod
    def _filter_texts_search(pattern, bitflag, texts_search, reverse=False):
        """
        Set FILTER_ITEM for texts search which text name matches filter_name one (case-insensitive).
        pattern is the filtering pattern.
        return a list of flags based on given bit flag, or an empty list if no pattern is given
        or texts search list is empty.
        """

        if not pattern or not texts_search:  # Empty pattern or list = no filtering!
            return []

        import fnmatch
        import re

        # Implicitly add heading/trailing wildcards.
        pattern_regex = re.compile(fnmatch.translate("*" + pattern + "*"), re.IGNORECASE)

        flags = [0] * len(texts_search)
        for i, text_search in enumerate(texts_search):
            name = text_search.text.name
            # This is similar to a logical XOR.
            if bool(name and pattern_regex.match(name)) is not reverse:
                flags[i] |= bitflag
        return flags

    @staticmethod
    def _sort_texts_search(texts_search):
        """
        Re-order text search using the text name (case-insensitive).
        return a list mapping org_idx -> new_idx, or an empty list if no sorting has been done.
        """
        _sort = [(idx, text_search.text.name) for idx, text_search in enumerate(texts_search)]
        return bpy.types.UI_UL_list.sort_items_helper(_sort, lambda e: e[1].lower())

    def filter_items(self, context, data, property):
        texts_search = getattr(data, property)
        flags = []
        indices = []

        # Filtering by text name
        if self.filter_name:
            flags = self._filter_texts_search(
                self.filter_name, self.bitflag_filter_item, texts_search, reverse=self.use_filter_invert)
        if not flags:
            flags = [self.bitflag_filter_item] * len(texts_search)

        for idx, text_search in enumerate(texts_search):
            # Filter text with no matches
            if len(text_search.string_matches) == 0:
                flags[idx] = 0
        if self.use_filter_sort_alpha:
            indices = self._sort_texts_search(texts_search)
        return flags, indices

    def draw_item(self, context, layout, _data, text_search, icon, _active_data, _active_propname, _index):
        row = layout.row()
        row.emboss = 'NONE'
        text_label = text_search.text.name
        if text_search.text == context.space_data.text:
            text_label = iface_("%s (current)") % text_label

        row.prop(text_search.text, "name", text="")
        row.label(text=str(len(text_search.string_matches)))
        if len(text_search.string_matches) == 0:
            return
        op = row.operator("text.open_text_with_selection", icon='ANIM', text="")
        op.text = text_search.text.name
        string_match = text_search.string_matches[0]
        op.start_line = string_match.line_index
        op.end_line = string_match.line_index
        op.start_sel = string_match.start
        op.end_sel = string_match.end


class TEX_UL_string_matches(bpy.types.UIList):
    def draw_item(self, context, layout, _data, string_match, icon, _active_data, _active_propname, _index):
        row = layout.row()
        row.prop(string_match, "select", text=str(string_match.line_index))
        row.label(text="..." + string_match.text_line.body.encode("utf8")[string_match.start:].decode('utf8'))
        row.emboss = 'NONE'
        op = row.operator("text.open_text_with_selection", icon='ANIM', text="")
        st = context.space_data
        op.text = st.text.name
        op.start_line = string_match.line_index
        op.end_line = string_match.line_index
        op.start_sel = string_match.start
        op.end_sel = string_match.end


class TEXT_PT_find(Panel):
    bl_space_type = 'TEXT_EDITOR'
    bl_region_type = 'UI'
    bl_category = "Text"
    bl_label = "Find & Replace"

    def draw(self, context):
        layout = self.layout
        st = context.space_data

        # find
        col = layout.column()
        row = col.row(align=True)
        row.prop(st, "find_text", text="", placeholder="Search", )
        row.prop(st, "use_match_case", icon='SORTALPHA', text="", text_ctxt=i18n_contexts.id_text)

        row.operator("text.find", icon='TRIA_UP', text="").previous = True
        row.operator("text.find", icon='TRIA_DOWN', text="")

        # replace
        col = layout.column()
        row = col.row(align=True)
        row.prop(st, "replace_text", placeholder="Replace", text="")

        row.operator("text.replace", icon='UV_SYNC_SELECT', text="")
        row.operator("text.replace", icon='CON_ROTLIKE', text="", text_ctxt="Replace all").all = True

        col = layout.column()
        row = col.row(align=True)
        row.prop(st, "use_find_all", text="Search in all text data-blocks")
        #if st.use_find_all:
        layout.template_list(
            "TEX_UL_texts_search",
            "",
            st,
            "texts_search",
            st,
            "active_text_search",
            rows=4,
        )
        if st.active_text_search != -1:
            layout.template_list(
                "TEX_UL_string_matches",
                "",
                st.texts_search[st.active_text_search],
                "string_matches",
                st,
                "active_string_match",
                rows=8,
            )


class TEXT_MT_view_navigation(Menu):
    bl_label = "Navigation"

    def draw(self, _context):
        layout = self.layout

        layout.operator("text.move", text="Top").type = 'FILE_TOP'
        layout.operator("text.move", text="Bottom").type = 'FILE_BOTTOM'

        layout.separator()

        layout.operator("text.move", text="Line Begin").type = 'LINE_BEGIN'
        layout.operator("text.move", text="Line End").type = 'LINE_END'

        layout.separator()

        layout.operator("text.move", text="Previous Line").type = 'PREVIOUS_LINE'
        layout.operator("text.move", text="Next Line").type = 'NEXT_LINE'

        layout.separator()

        layout.operator("text.move", text="Previous Word").type = 'PREVIOUS_WORD'
        layout.operator("text.move", text="Next Word").type = 'NEXT_WORD'


class TEXT_MT_view(Menu):
    bl_label = "View"

    def draw(self, context):
        layout = self.layout

        st = context.space_data

        layout.prop(st, "show_region_ui")

        layout.separator()

        layout.prop(st, "show_line_numbers")
        layout.prop(st, "show_word_wrap")
        syntax = layout.column()
        syntax.active = st.is_syntax_highlight_supported()
        syntax.prop(st, "show_syntax_highlight")
        layout.prop(st, "show_line_highlight")

        layout.separator()

        props = layout.operator("wm.context_cycle_int", text="Zoom In")
        props.data_path = "space_data.font_size"
        props.reverse = False

        props = layout.operator("wm.context_cycle_int", text="Zoom Out")
        props.data_path = "space_data.font_size"
        props.reverse = True

        layout.separator()

        layout.menu("TEXT_MT_view_navigation")

        layout.separator()

        layout.menu("INFO_MT_area")


class TEXT_MT_text(Menu):
    bl_label = "Text"

    def draw(self, context):
        layout = self.layout

        st = context.space_data
        text = st.text

        layout.operator("text.new", text="New",
                        text_ctxt=i18n_contexts.id_text, icon='FILE_NEW')
        layout.operator("text.open", text="Open...", icon='FILE_FOLDER')

        if text:
            layout.separator()
            row = layout.row()
            row.operator("text.reload")
            row.enabled = not text.is_in_memory

            row = layout.row()
            row.operator("text.jump_to_file_at_point", text="Edit Externally")
            row.enabled = (not text.is_in_memory and context.preferences.filepaths.text_editor != "")

            layout.separator()
            layout.operator("text.save", icon='FILE_TICK')
            layout.operator("text.save_as", text="Save As...")

            if text.filepath:
                layout.separator()
                layout.operator("text.make_internal")

            layout.separator()
            layout.prop(text, "use_module")

            layout.prop(st, "use_live_edit")

            layout.separator()
            layout.operator("text.run_script")


class TEXT_MT_templates_py(Menu):
    bl_label = "Python"

    def draw(self, _context):
        self.path_menu(
            bpy.utils.script_paths(subdir="templates_py"),
            "text.open",
            props_default={"internal": True},
            filter_ext=lambda ext: (ext.lower() == ".py"),
        )


class TEXT_MT_templates_osl(Menu):
    bl_label = "Open Shading Language"

    def draw(self, _context):
        self.path_menu(
            bpy.utils.script_paths(subdir="templates_osl"),
            "text.open",
            props_default={"internal": True},
            filter_ext=lambda ext: (ext.lower() == ".osl"),
        )


class TEXT_MT_templates(Menu):
    bl_label = "Templates"

    def draw(self, _context):
        layout = self.layout
        layout.menu("TEXT_MT_templates_py")
        layout.menu("TEXT_MT_templates_osl")


class TEXT_MT_select(Menu):
    bl_label = "Select"

    def draw(self, _context):
        layout = self.layout

        layout.operator("text.select_all", text="All")
        layout.operator("text.select_line", text="Line")
        layout.operator("text.select_word", text="Word")

        layout.separator()

        layout.operator("text.move_select", text="Top").type = 'FILE_TOP'
        layout.operator("text.move_select", text="Bottom").type = 'FILE_BOTTOM'

        layout.separator()

        layout.operator("text.move_select", text="Line Begin").type = 'LINE_BEGIN'
        layout.operator("text.move_select", text="Line End").type = 'LINE_END'

        layout.separator()

        layout.operator("text.move_select", text="Previous Line").type = 'PREVIOUS_LINE'
        layout.operator("text.move_select", text="Next Line").type = 'NEXT_LINE'

        layout.separator()

        layout.operator("text.move_select", text="Previous Word").type = 'PREVIOUS_WORD'
        layout.operator("text.move_select", text="Next Word").type = 'NEXT_WORD'


class TEXT_MT_format(Menu):
    bl_label = "Format"

    def draw(self, _context):
        layout = self.layout

        layout.operator("text.indent")
        layout.operator("text.unindent")

        layout.separator()

        layout.operator("text.comment_toggle")

        layout.separator()

        layout.operator_menu_enum("text.convert_whitespace", "type")


class TEXT_MT_edit_to3d(Menu):
    bl_label = "Text to 3D Object"

    def draw(self, _context):
        layout = self.layout

        layout.operator("text.to_3d_object",
                        text="One Object",
                        ).split_lines = False
        layout.operator("text.to_3d_object",
                        text="One Object Per Line",
                        ).split_lines = True


class TEXT_MT_edit(Menu):
    bl_label = "Edit"

    @classmethod
    def poll(cls, context):
        return context.space_data.text is not None

    def draw(self, _context):
        layout = self.layout

        layout.operator("ed.undo")
        layout.operator("ed.redo")

        layout.separator()

        layout.operator("text.cut")
        layout.operator("text.copy", icon='COPYDOWN')
        layout.operator("text.paste", icon='PASTEDOWN')
        layout.operator("text.duplicate_line")

        layout.separator()

        layout.operator("text.move_lines", text="Move Line(s) Up").direction = 'UP'
        layout.operator("text.move_lines", text="Move Line(s) Down").direction = 'DOWN'

        layout.separator()

        layout.operator("text.start_find", text="Find & Replace...")
        layout.operator("text.find_set_selected")
        layout.operator("text.jump", text="Jump To...")

        layout.separator()

        layout.operator("text.autocomplete")

        layout.separator()

        layout.menu("TEXT_MT_edit_to3d")


class TEXT_MT_context_menu(Menu):
    bl_label = ""

    def draw(self, _context):
        layout = self.layout

        layout.operator_context = 'INVOKE_DEFAULT'

        layout.operator("text.cut")
        layout.operator("text.copy", icon='COPYDOWN')
        layout.operator("text.paste", icon='PASTEDOWN')
        layout.operator("text.duplicate_line")

        layout.separator()

        layout.operator("text.move_lines", text="Move Line(s) Up").direction = 'UP'
        layout.operator("text.move_lines", text="Move Line(s) Down").direction = 'DOWN'

        layout.separator()

        layout.operator("text.indent")
        layout.operator("text.unindent")

        layout.separator()

        layout.operator("text.comment_toggle")

        layout.separator()

        layout.operator("text.autocomplete")


classes = (
    TEXT_HT_header,
    TEXT_HT_footer,
    TEXT_MT_edit,
    TEXT_MT_editor_menus,
    TEXT_PT_find,
    TEXT_PT_properties,
    TEXT_MT_view,
    TEXT_MT_view_navigation,
    TEXT_MT_text,
    TEXT_MT_templates,
    TEXT_MT_templates_py,
    TEXT_MT_templates_osl,
    TEXT_MT_select,
    TEXT_MT_format,
    TEXT_MT_edit_to3d,
    TEXT_MT_context_menu,
    TEX_UL_texts_search,
    TEX_UL_string_matches,
)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
