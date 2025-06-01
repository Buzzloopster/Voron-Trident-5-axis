bl_info = {
    "name": "AutoCut GCode for 3+2 Axis",
    "author": "AI Assistant (Jules)",
    "version": (0, 2, 0), # Version bump for new features
    "blender": (2, 80, 0),
    "location": "View3D > Sidebar > AutoCut Tab",
    "description": "Performs Boolean cuts, slices for multiple orientations, and generates G-code for 3+2 axis machining.",
    "warning": "",
    "doc_url": "", # TODO: Add documentation URL
    "category": "3D Print",
}

import bpy

# Import submodules
from . import properties
from . import operators
from . import ui
from . import slicer
from . import gcode_writer # Ensure gcode_writer is imported

modules = [
    properties,
    operators,
    ui,
    slicer,
    gcode_writer, # Add gcode_writer to the list
]

def register():
    print("Registering AutoCut GCode Plugin Modules...")
    for mod in modules:
        if hasattr(mod, 'register'):
            try:
                mod.register()
            except Exception as e:
                print(f"Error registering module {mod.__name__}: {e}")
        # Modules like slicer or gcode_writer might just contain classes/functions
        # and not have their own register() if they don't add UI elements or specific bpy.types

    # Initialize custom scene properties (idempotent checks)
    if not hasattr(bpy.types.Scene, 'autocut_slicing_settings'):
        # This should be created by properties.py, but as a fallback:
        bpy.types.Scene.autocut_slicing_settings = bpy.props.PointerProperty(type=properties.SlicingSettings)
        print("Fallback: Initialized Scene.autocut_slicing_settings in __init__")

    if not hasattr(bpy.types.Scene, 'selected_planars_data'):
        bpy.types.Scene.selected_planars_data = []

    if not hasattr(bpy.types.Scene, 'all_toolpath_data'):
        bpy.types.Scene.all_toolpath_data = []

    print("AutoCut GCode Plugin Registered.")

def unregister():
    print("Unregistering AutoCut GCode Plugin Modules...")
    for mod in reversed(modules):
        if hasattr(mod, 'unregister'):
            try:
                mod.unregister()
            except Exception as e:
                print(f"Error unregistering module {mod.__name__}: {e}")

    # Clean up custom scene properties
    # properties.py should handle autocut_slicing_settings
    if hasattr(bpy.types.Scene, 'selected_planars_data'):
        del bpy.types.Scene.selected_planars_data

    if hasattr(bpy.types.Scene, 'all_toolpath_data'):
        del bpy.types.Scene.all_toolpath_data

    # Deleting autocut_slicing_settings should ideally be handled by properties.py unregister
    # to keep things modular, but as a safeguard:
    if hasattr(bpy.types.Scene, 'autocut_slicing_settings') and not hasattr(properties, 'unregister'):
         print("Warning: properties.py has no unregister, but scene.autocut_slicing_settings exists.")
         # Consider del bpy.types.Scene.autocut_slicing_settings here if properties.py can't.
         # However, properties.py *does* have unregister.

    print("AutoCut GCode Plugin Unregistered.")

if __name__ == "__main__":
    # This part is mainly for direct script execution in Blender's text editor for development
    try:
        unregister()
    except Exception as e:
        pass
    register()
