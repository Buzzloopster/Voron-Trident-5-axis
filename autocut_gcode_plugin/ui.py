# autocut_gcode_plugin/ui.py
import bpy

class AUTOCUT_PT_MainPanel(bpy.types.Panel):
    """Creates a Panel in the 3D Viewport N-panel"""
    bl_label = "AutoCut GCode"
    bl_idname = "AUTOCUT_PT_MainPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'AutoCut'

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        slicing_props = None
        if hasattr(scene, 'autocut_slicing_settings'):
            slicing_props = scene.autocut_slicing_settings
        else:
            layout.label(text="Slicing settings not loaded!", icon='ERROR')
            return

        layout.label(text="Plugin Workflow", icon='TOOL_SETTINGS')

        # Workflow Buttons
        row = layout.row()
        row.operator("autocut.select_planars_operator", text="1. Process Selected Planar Faces")

        row = layout.row()
        planar_data_exists = hasattr(bpy.types.Scene, 'selected_planars_data') and bpy.types.Scene.selected_planars_data

        op_boolcut = row.operator("autocut.perform_boolean_cuts_operator", text="2. Perform Boolean Cuts")
        op_boolcut.active = planar_data_exists # Disable if no planar data

        row = layout.row()
        # Check if WCS & Slice data exists (all_toolpath_data might be a better check later)
        gcode_data_exists = hasattr(bpy.types.Scene, 'all_toolpath_data') and bpy.types.Scene.all_toolpath_data
        prev_steps_done = planar_data_exists # For now, just require planar data for the "data calculation" step

        op_calc_gcode_data = row.operator("autocut.generate_gcode_operator", text="3. Calculate WCS & Slice Data")
        op_calc_gcode_data.active = planar_data_exists

        # Actual G-Code File Export Button - this will be the new one
        # For now, let's assume the 'Calculate WCS & Slice Data' button will eventually be 'Export G-Code File'
        # and will trigger the ExportHelper. The label was changed in the previous step in anticipation.
        # The operator bl_label will be "Export G-Code File" after the next step.

        layout.separator()
        layout.label(text="Slicing Settings:")

        if slicing_props:
            col = layout.column(align=True)
            col.prop(slicing_props, "layer_height")
            col.prop(slicing_props, "nozzle_diameter")
            col.prop(slicing_props, "wall_thickness")
            col.prop(slicing_props, "infill_density")

            layout.separator()
            sub_col_feed = col.column(align=True) # Group feed rates
            sub_col_feed.label(text="Print/Travel Feed Rates:")
            sub_col_feed.prop(slicing_props, "feed_rate_print")
            sub_col_feed.prop(slicing_props, "feed_rate_travel")
            sub_col_feed.prop(slicing_props, "feed_rate_rotation")


            layout.separator()
            layout.label(text="G-Code & Extrusion Settings:")
            gcode_col = layout.column(align=True)
            gcode_col.prop(slicing_props, "start_wcs_p_number")
            gcode_col.prop(slicing_props, "gcode_axis_A_letter")
            gcode_col.prop(slicing_props, "gcode_axis_C_letter")

            gcode_col.separator() # Visual break before extrusion params
            gcode_col.prop(slicing_props, "filament_diameter")
            gcode_col.prop(slicing_props, "extrusion_multiplier")

            gcode_col.separator() # Visual break for retraction/z-hop
            gcode_col.prop(slicing_props, "retraction_amount")
            gcode_col.prop(slicing_props, "retraction_speed")
            gcode_col.prop(slicing_props, "z_hop_height")

            layout.separator()
            box = layout.box()
            row = box.row()
            row.alignment = 'CENTER'
            row.label(text="Custom G-Code Snippets")

            box.prop(slicing_props, "gcode_header", text="Header") # Removed redundant text=""
            box.prop(slicing_props, "gcode_footer", text="Footer")
            box.prop(slicing_props, "gcode_before_orientation_change", text="Before Orientation")
            box.prop(slicing_props, "gcode_after_orientation_change", text="After Orientation")
        else:
            layout.label(text="Slicing settings are missing.", icon='ERROR')

        # Boolean Cut settings note (as before)
        layout.separator()
        layout.label(text="Boolean Cut Settings (use F9 after clicking cut):")
        box_bool = layout.box()
        box_bool.label(text="Default Cutter Size: 100.0")
        box_bool.label(text="Default Cutter Thickness: 0.1")

classes = [
    AUTOCUT_PT_MainPanel,
]

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
