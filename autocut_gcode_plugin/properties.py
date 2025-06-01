# autocut_gcode_plugin/properties.py
import bpy
import math

class SlicingSettings(bpy.types.PropertyGroup):
    # Existing Slicing properties
    layer_height: bpy.props.FloatProperty(
        name="Layer Height (mm)", default=0.2, min=0.01, max=2.0, unit='LENGTH', precision=3
    )
    nozzle_diameter: bpy.props.FloatProperty(
        name="Nozzle Diameter (mm)", default=0.4, min=0.1, max=2.0, unit='LENGTH', precision=3
    )
    infill_density: bpy.props.FloatProperty( # Placeholder, actual infill logic not implemented yet
        name="Infill Density", default=0.2, min=0.0, max=1.0, subtype='FACTOR', precision=2
    )
    wall_thickness: bpy.props.FloatProperty( # Placeholder, actual wall logic not implemented yet
        name="Wall Thickness (mm)", default=0.8, min=0.1, unit='LENGTH', precision=3
    )

    feed_rate_print: bpy.props.FloatProperty(
        name="Print Feed Rate (mm/min)", default=1800.0, min=1.0
    )
    feed_rate_travel: bpy.props.FloatProperty(
        name="Travel Feed Rate (mm/min)", default=3000.0, min=1.0
    )
    feed_rate_rotation: bpy.props.FloatProperty(
        name="Rotation Feed Rate (deg/min)",
        description="Feed rate for rotary axis movements (A, C)",
        default=5000.0, min=1.0
    )

    # WCS and G-Code Axis Letters
    start_wcs_p_number: bpy.props.IntProperty(
        name="Start WCS P-Number", default=1, min=1, max=255
    )
    gcode_axis_A_letter: bpy.props.StringProperty(
        name="A-Axis Letter", default="A"
    )
    gcode_axis_C_letter: bpy.props.StringProperty(
        name="C-Axis Letter", default="C"
    )

    # New properties for G-Code Generation
    filament_diameter: bpy.props.FloatProperty(
        name="Filament Diameter (mm)",
        default=1.75,
        min=0.1,
        unit='LENGTH',
        precision=3
    )
    extrusion_multiplier: bpy.props.FloatProperty(
        name="Extrusion Multiplier",
        description="Multiplier for calculated extrusion amount (e.g., 1.0 for 100%)",
        default=1.0,
        min=0.01, # Avoid zero if calculation is direct multiplication
        max=3.0,
        subtype='FACTOR',
        precision=3
    )
    retraction_amount: bpy.props.FloatProperty(
        name="Retraction Amount (mm)",
        description="Amount of filament to retract (positive value)",
        default=2.0, min=0.0, precision=3
    )
    retraction_speed: bpy.props.FloatProperty(
        name="Retraction Speed (mm/min)", default=1800.0, min=1.0
    )
    z_hop_height: bpy.props.FloatProperty(
        name="Z Hop Height (mm)",
        description="Height to lift Z during travel moves over printed areas",
        default=0.5, min=0.0, precision=3
    )

    gcode_header: bpy.props.StringProperty(
        name="G-Code Header",
        description="Custom G-code to prepend at the beginning of the file. Use '\\n' for newlines.",
        default="G21 ; Set units to millimeters\nG90 ; Use absolute positioning\nM82 ; Use absolute E distances\nG92 E0 ; Reset extruder position",
        subtype='TEXTAREA' # Makes it a multi-line input box
    )
    gcode_footer: bpy.props.StringProperty(
        name="G-Code Footer",
        description="Custom G-code to append at the end of the file. Use '\\n' for newlines.",
        default="M104 S0 ; Turn off extruder heater\nM140 S0 ; Turn off bed heater\nG91 ; Relative positioning\nG1 Z10 F3000 ; Move Z up\nG90 ; Absolute positioning\nG28 X0 Y0 ; Home X and Y\nM84 ; Disable motors",
        subtype='TEXTAREA'
    )
    gcode_before_orientation_change: bpy.props.StringProperty(
        name="Before Orientation Change",
        description="G-code to run before rotating A/C axes. Placeholders: {safe_z}, {travel_feedrate}, {retraction_amount}, {retraction_speed}",
        default="G1 E-{retraction_amount} F{retraction_speed} ; Retract filament\nG1 Z{safe_z} F{travel_feedrate} ; Move to safe Z",
        subtype='TEXTAREA'
    )
    gcode_after_orientation_change: bpy.props.StringProperty(
        name="After Orientation Change",
        description="G-code to run after rotating A/C and setting WCS. Placeholders: {travel_feedrate}, {retraction_amount}, {retraction_speed} (for priming/unretracting)",
        default="G1 X0 Y0 F{travel_feedrate} ; Move to new WCS origin XY (relative to current WCS)\nG1 E{retraction_amount} F{retraction_speed} ; Unretract/Prime filament\nG92 E0 ; Reset extruder for this orientation",
        subtype='TEXTAREA'
    )

classes = [ SlicingSettings ]

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.autocut_slicing_settings = bpy.props.PointerProperty(type=SlicingSettings)

def unregister():
    if hasattr(bpy.types.Scene, 'autocut_slicing_settings'):
        del bpy.types.Scene.autocut_slicing_settings
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
