# autocut_gcode_plugin/operators.py
import bpy
import bmesh # Still needed for is_face_planar if it's here, or SelectPlanars
import mathutils
import math
from bpy_extras.io_utils import ExportHelper # For file dialog
from bpy.props import StringProperty # No BoolProperty needed for now

from . import slicer
from . import gcode_writer # Import the new writer module

# --- is_face_planar ---
# (Assuming this function is as defined in previous steps and is working correctly)
def is_face_planar(bm_face, tolerance=1e-5):
    if len(bm_face.verts) < 3: return True
    v1 = bm_face.verts[0].co
    v2 = bm_face.verts[1].co
    v3 = bm_face.verts[2].co
    cross_product = (v2 - v1).cross(v3 - v1)
    if cross_product.length < tolerance:
        if len(bm_face.verts) > 2:
            v_dir = (v2 - v1).normalized_safe()
            if v_dir.length == 0 and len(bm_face.verts) > 1:
                 v_dir = (bm_face.verts[len(bm_face.verts)-1].co - v1).normalized_safe()
            if v_dir.length == 0: return True
            for i in range(2, len(bm_face.verts)):
                if (bm_face.verts[i].co - v1).normalized_safe().cross(v_dir).length > tolerance:
                    return False
        return True
    face_normal = cross_product.normalized()
    for i in range(3, len(bm_face.verts)):
        vn = bm_face.verts[i].co
        if abs((vn - v1).dot(face_normal)) > tolerance:
            return False
    return True

# --- AUTOCUT_OT_SelectPlanars ---
class AUTOCUT_OT_SelectPlanars(bpy.types.Operator):
    bl_idname = "autocut.select_planars_operator"
    bl_label = "Process Selected Planar Faces"
    bl_options = {'REGISTER', 'UNDO'}
    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'MESH':
            self.report({'WARNING'}, "Active object is not a mesh.")
            return {'CANCELLED'}
        if context.mode != 'EDIT_MESH':
            self.report({'WARNING'}, "Please be in Edit Mode to select faces.")
            return {'CANCELLED'}
        bm = bmesh.from_edit_mesh(obj.data)
        bm.normal_update()
        selected_faces = [f for f in bm.faces if f.select]
        if not selected_faces:
            self.report({'INFO'}, "No faces selected.")
            return {'CANCELLED'}
        if not hasattr(bpy.types.Scene, 'selected_planars_data'):
             bpy.types.Scene.selected_planars_data = []
        bpy.types.Scene.selected_planars_data.clear()
        valid_planars_found = 0
        for face in selected_faces:
            if is_face_planar(face):
                normal = face.normal.copy()
                center = face.calc_center_median()
                obj_mat_world = obj.matrix_world
                try:
                    mat_world_inv_transp = obj_mat_world.inverted_safe().transpose()
                    world_normal = (mat_world_inv_transp.to_3x3() @ normal).normalized_safe()
                except ValueError:
                    self.report({'WARNING'}, f"Object matrix for face {face.index} not invertible. Using local normal.")
                    world_normal = normal.normalized_safe()
                if world_normal.length < 0.1:
                    self.report({'WARNING'}, f"Face {face.index} has a near-zero normal. Skipping.")
                    continue
                world_center = obj_mat_world @ center
                bpy.types.Scene.selected_planars_data.append({
                    'center': world_center.to_tuple(), # Store as tuple for JSON serializability if ever needed
                    'normal': world_normal.to_tuple(), # Store as tuple
                    'original_face_index': face.index
                })
                valid_planars_found += 1
            else:
                self.report({'WARNING'}, f"Face {face.index} is not planar (or degenerate) and will be ignored.")
        if valid_planars_found == 0:
            self.report({'WARNING'}, "No valid planar faces were selected or processed.")
            return {'CANCELLED'}
        self.report({'INFO'}, f"Processed {valid_planars_found} planar face(s). Ready for cutting/G-code data calculation.")
        return {'FINISHED'}

# --- AUTOCUT_OT_PerformBooleanCuts ---
class AUTOCUT_OT_PerformBooleanCuts(bpy.types.Operator):
    bl_idname = "autocut.perform_boolean_cuts_operator"
    bl_label = "Perform Boolean Cuts"
    bl_options = {'REGISTER', 'UNDO'}
    cutter_size: bpy.props.FloatProperty(name="Cutter Size", default=100.0, min=0.1)
    cutter_thickness: bpy.props.FloatProperty(name="Cutter Thickness", default=0.1, min=0.001)
    def execute(self, context):
        target_obj = context.view_layer.objects.active
        if not target_obj or target_obj.type != 'MESH':
            self.report({'WARNING'}, "No active mesh object selected to cut.")
            return {'CANCELLED'}
        if not hasattr(bpy.types.Scene, 'selected_planars_data') or not bpy.types.Scene.selected_planars_data:
            self.report({'WARNING'}, "No planar data found. Process planar faces first.")
            return {'CANCELLED'}
        if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
        planar_data_list = bpy.types.Scene.selected_planars_data
        if bpy.ops.object.select_all.poll(): bpy.ops.object.select_all(action='DESELECT')
        target_obj.select_set(True)
        context.view_layer.objects.active = target_obj
        successful_cuts = 0
        for i, planar_data_dict in enumerate(planar_data_list):
            planar_data = {k: mathutils.Vector(v) if k in ['center', 'normal'] else v for k,v in planar_data_dict.items()} # Convert tuples back to vectors for use
            if not all(k in planar_data for k in ['center', 'normal']):
                self.report({'WARNING'}, f"Planar data for item {i} is malformed. Skipping.")
                continue
            plane_center = planar_data['center']
            plane_normal = planar_data['normal']
            if plane_normal.length < 0.001:
                self.report({'WARNING'}, f"Planar normal for item {i} is zero or too small. Skipping cut.")
                continue
            bpy.ops.mesh.primitive_cube_add(size=1.0, enter_editmode=False, align='WORLD', location=(0,0,0))
            cutter_obj = context.active_object
            cutter_obj.name = f"Temporary_Cutter_{i}"
            cutter_obj.scale = (self.cutter_size, self.cutter_size, self.cutter_thickness)
            bpy.ops.object.transform_apply(location=False, rotation=False, scale=True, properties=False)
            rot_quat = plane_normal.to_track_quat('Z', 'Y')
            cutter_obj.rotation_euler = rot_quat.to_euler()
            cutter_obj.location = plane_center + plane_normal * (self.cutter_thickness / 2.0)
            context.view_layer.objects.active = target_obj
            target_obj.select_set(True)
            bool_mod = target_obj.modifiers.new(name=f"BooleanCut_{i}", type='BOOLEAN')
            bool_mod.operation = 'DIFFERENCE'
            bool_mod.object = cutter_obj
            try:
                bpy.ops.object.modifier_apply(modifier=bool_mod.name)
                successful_cuts +=1
            except RuntimeError as e:
                self.report({'ERROR'}, f"Boolean cut for plane {i+1} failed: {e}. Skipping.")
                if bool_mod.name in target_obj.modifiers: target_obj.modifiers.remove(bool_mod)
            finally:
                cutter_name = cutter_obj.name
                bpy.ops.object.select_all(action='DESELECT')
                if cutter_name in bpy.data.objects:
                    bpy.data.objects[cutter_name].select_set(True)
                    bpy.ops.object.delete()
            if target_obj.name not in bpy.data.objects:
                self.report({'ERROR'}, "Target object was deleted or is no longer available.")
                return {'CANCELLED'}
            target_obj.select_set(True)
            context.view_layer.objects.active = target_obj
        self.report({'INFO'}, f"Attempted {len(planar_data_list)} Boolean cut(s). {successful_cuts} succeeded.")
        return {'FINISHED'}

# --- AUTOCUT_OT_GenerateGcode (Now for data calculation AND export) ---
class AUTOCUT_OT_GenerateGcode(bpy.types.Operator, ExportHelper): # Inherit ExportHelper
    """Calculates WCS, Slices, and Exports G-Code for 3+2 Axis Operations"""
    bl_idname = "autocut.generate_gcode_operator"
    bl_label = "Export G-Code File" # Updated label for the button
    bl_options = {'REGISTER', 'UNDO'}

    # ExportHelper mixin class properties
    filename_ext = ".gcode" # Default file extension
    filepath: StringProperty(
        name="File Path",
        description="File path for exporting the G-code file",
        maxlen=1024,
        subtype='FILE_PATH', # Opens file browser
    )
    # filter_glob: StringProperty(default="*.gcode;*.nc;*.txt", options={'HIDDEN'}) # Optional filter

    # Internal flag to manage if data calculation is needed
    # This is a bit of a workaround for a single button doing two conceptual steps.
    # A better UI might have separate buttons for "Calculate Data" and "Export Last Calculated".
    # For now, this operator will always recalculate.

    def invoke(self, context, event): # Standard ExportHelper method
        # Pre-fill filename based on active object name or blend file name
        default_filename = "output.gcode"
        if context.blend_data.filepath:
            blend_name = bpy.path.basename(context.blend_data.filepath)
            default_filename = bpy.path.display_name_from_filepath(blend_name) + ".gcode"
        if context.active_object:
            default_filename = context.active_object.name.replace(".","_") + ".gcode"

        self.filepath = bpy.path.ensure_ext(default_filename, self.filename_ext)
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        target_obj = context.active_object
        if not target_obj or target_obj.type != 'MESH':
            self.report({'WARNING'}, "Active object is not a mesh.")
            return {'CANCELLED'}
        if not hasattr(bpy.types.Scene, 'selected_planars_data') or not bpy.types.Scene.selected_planars_data:
            self.report({'WARNING'}, "No planar data found. Please use 'Process Selected Planar Faces' first.")
            return {'CANCELLED'}
        if not hasattr(context.scene, 'autocut_slicing_settings'):
            self.report({'ERROR'}, "Slicing settings not found! Check addon registration.")
            return {'CANCELLED'}

        slice_settings = context.scene.autocut_slicing_settings
        self.report({'INFO'}, f"Starting G-code data calculation for {target_obj.name}...")

        if hasattr(bpy.types.Scene, 'all_toolpath_data'):
            bpy.types.Scene.all_toolpath_data.clear()
        else:
            bpy.types.Scene.all_toolpath_data = [] # Ensure it exists

        current_wcs_p_number = slice_settings.start_wcs_p_number
        processed_orientations_data = []

        for planar_idx, planar_info_dict in enumerate(bpy.types.Scene.selected_planars_data):
            # Convert stored tuples back to Vectors for math
            plane_normal_world = mathutils.Vector(planar_info_dict['normal']).normalized_safe()
            plane_center_world = mathutils.Vector(planar_info_dict['center'])

            if plane_normal_world.length < 0.1:
                self.report({'WARNING'}, f"Skipping orientation {planar_idx+1} due to invalid normal.")
                continue
            self.report({'INFO'}, f"Processing WCS & Slicing for orientation {planar_idx+1}")

            Nx, Ny, Nz = plane_normal_world.x, plane_normal_world.y, plane_normal_world.z
            if math.isclose(Nx, 0.0, abs_tol=1e-6) and math.isclose(Ny, 0.0, abs_tol=1e-6):
                C_deg = 0.0
                A_deg = 0.0 if Nz > 0 else 180.0
            else:
                C_rad = math.atan2(Nx, Ny)
                A_rad = math.acos(Nz)
                C_deg = math.degrees(C_rad)
                A_deg = math.degrees(A_rad)

            wcs_rotation_ac_deg = (A_deg, C_deg)
            wcs_origin_xyz = plane_center_world

            rot_to_z_up = plane_normal_world.rotation_difference(mathutils.Vector((0,0,1.0))).to_matrix().to_4x4()
            center_in_rotated_frame = rot_to_z_up @ plane_center_world
            trans_to_origin = mathutils.Matrix.Translation(-center_in_rotated_frame)
            world_to_slicing_frame_transform = trans_to_origin @ rot_to_z_up

            layers_for_this_orientation = slicer.generate_toolpaths_for_orientation(
                target_obj, world_to_slicing_frame_transform, slice_settings, context
            )

            if layers_for_this_orientation:
                processed_orientations_data.append({
                    'original_plane_index': planar_idx,
                    'wcs_p_number': current_wcs_p_number,
                    'wcs_gcode_axes': (slice_settings.gcode_axis_A_letter, slice_settings.gcode_axis_C_letter),
                    'wcs_origin_xyz': wcs_origin_xyz.to_tuple(), # Store as tuple
                    'wcs_rotation_ac_deg': wcs_rotation_ac_deg, # Tuple (A, C)
                    'world_to_slicing_frame_transform': [list(row) for row in world_to_slicing_frame_transform], # Store as list of lists
                    'slicing_frame_to_world_transform': [list(row) for row in world_to_slicing_frame_transform.inverted_safe()],
                    'layers': layers_for_this_orientation
                })
                current_wcs_p_number += 1
            else:
                self.report({'WARNING'}, f"  No toolpaths generated for orientation {planar_idx+1}.")

        bpy.types.Scene.all_toolpath_data = processed_orientations_data
        if not bpy.types.Scene.all_toolpath_data:
            self.report({'WARNING'}, "G-code data calculation failed to produce any toolpaths.")
            return {'CANCELLED'}
        self.report({'INFO'}, f"G-code data calculation complete for {len(bpy.types.Scene.all_toolpath_data)} orientations.")

        # --- Now, proceed to G-Code file export ---
        self.report({'INFO'}, f"Exporting G-code to: {self.filepath}")
        exporter = gcode_writer.GCodeExporter(
            bpy.types.Scene.all_toolpath_data, # This now contains the fully processed data
            slice_settings,
            context.scene
        )

        if exporter.export_gcode(self.filepath):
            self.report({'INFO'}, f"G-code successfully exported to {self.filepath}")
        else:
            self.report({'ERROR'}, f"Failed to export G-code to {self.filepath}")
            return {'CANCELLED'}

        return {'FINISHED'}

classes = [
    AUTOCUT_OT_SelectPlanars,
    AUTOCUT_OT_PerformBooleanCuts,
    AUTOCUT_OT_GenerateGcode, # This is now the combined data calc + export operator
]

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
