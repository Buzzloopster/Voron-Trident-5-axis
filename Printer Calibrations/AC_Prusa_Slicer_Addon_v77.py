import bpy
import math
from math import atan2, pi, cos, sin, acos
import bmesh
import mathutils
import os
import subprocess
from bpy.props import StringProperty, FloatProperty, CollectionProperty, PointerProperty, BoolProperty

bl_info = {
    "name": "5-Axis FDM Slicer (AC Kinematics, PrusaSlicer)",
    "author": "Matej Krok, Modified by Community",
    "description": "A 5-axis slicing addon using AC kinematic model and PrusaSlicer.",
    "blender": (4, 3, 0),
    "version": (2, 2, 0), # G10 work offset kinematics fix
    "location": "3D View > UI > 5-Axis Printer",
    "warning": "",
    "category": "Generic",
}

# -------------------------------------------------------------------
# (2) Slicing Cubes Data (Collection Property)
# -------------------------------------------------------------------
class SlicingCubeItem(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(default="SlicingCube")
    isProcessed: bpy.props.BoolProperty(default=False, description="Has this slicing cube been processed?")

# -------------------------------------------------------------------
# (3) UIList for Slicing Cubes
# -------------------------------------------------------------------
class SLICINGCUBE_UL_items(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index=0, flt_flag=0):
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            layout.label(text=item.name, icon='CUBE')
        elif self.layout_type == 'GRID':
            layout.alignment = 'CENTER'
            layout.label(text="", icon='CUBE')

# -------------------------------------------------------------------
# (4) Add/Remove Slicing Cubes
# -------------------------------------------------------------------
class SLICINGCUBE_OT_add(bpy.types.Operator):
    bl_idname = "slicingcube.add"
    bl_label = "Add Slicing Cube"

    def execute(self, context):
        scene = context.scene
        bpy.ops.mesh.primitive_cube_add(size=50.0, location=(0, 0, 25))
        new_cube = context.active_object
        new_cube.name = self.unique_name("SlicingCube")
        new_cube.display_type = 'WIRE'
        new_cube.show_axis = True
        new_cube.empty_display_size = 120
        axis_length = 40.0
        axis_data = [
            ('X', (axis_length, 0, 0), (1, 0, 0, 1)),
            ('Y', (0, axis_length, 0), (0, 1, 0, 1)),
            ('Z', (0, 0, axis_length), (0, 0, 1, 1)),
        ]
        for axis, loc, color in axis_data:
            empty = bpy.data.objects.new(f"{new_cube.name}_AXIS_{axis}", None)
            empty.empty_display_type = 'ARROWS'
            empty.empty_display_size = axis_length
            empty.location = (0, 0, 0)
            empty.parent = new_cube
            empty.matrix_parent_inverse = new_cube.matrix_world.inverted()
            empty.show_in_front = True
            if hasattr(empty, "color"):
                empty.color = color
            scene.collection.objects.link(empty)
        scene.cursor.location = (0,0,0)
        bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
        scene.cursor.location = (0,0,scene.build_plate_distance)
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='DESELECT')
        obj = new_cube
        mesh = obj.data
        bm = bmesh.from_edit_mesh(mesh)
        for face in bm.faces:
            face.select = False
        max_z = -float('inf')
        top_faces = []
        for face in bm.faces:
            face_z = sum(v.co.z for v in face.verts) / len(face.verts)
            if face_z > max_z:
                max_z = face_z
                top_faces = [face]
            elif face_z == max_z:
                top_faces.append(face)
        for face in top_faces:
            face.select = True
        if top_faces:
            new_cube["top_face_index"] = top_faces[0].index
        bmesh.update_edit_mesh(mesh)
        bpy.ops.mesh.extrude_region_move(
            TRANSFORM_OT_translate={"value": (0, 0, 15), "orient_type": 'GLOBAL'}
        )
        bpy.ops.mesh.merge(type='CENTER')
        bpy.ops.object.mode_set(mode='OBJECT')
        item = scene.slicing_cubes_collection.add()
        item.name = new_cube.name
        scene.slicing_cubes_index = len(scene.slicing_cubes_collection) - 1
        self.report({'INFO'}, f"Added slicing cube: {new_cube.name}")
        return {'FINISHED'}

    def unique_name(self, base):
        existing = {o.name for o in bpy.data.objects}
        name = base
        i = 1
        while name in existing:
            name = f"{base}.{i:03d}"
            i += 1
        return name

class SLICINGCUBE_OT_remove(bpy.types.Operator):
    bl_idname = "slicingcube.remove"
    bl_label = "Remove Slicing Cube"

    @classmethod
    def poll(cls, context):
        scene = context.scene
        return (len(scene.slicing_cubes_collection) > 0 and 0 <= scene.slicing_cubes_index < len(scene.slicing_cubes_collection))

    def execute(self, context):
        scene = context.scene
        idx = scene.slicing_cubes_index
        item = scene.slicing_cubes_collection[idx]
        cube_obj = bpy.data.objects.get(item.name)
        if cube_obj:
            for obj in list(bpy.data.objects):
                if obj.parent == cube_obj and obj.type == 'EMPTY' and obj.name.startswith(f"{cube_obj.name}_AXIS_"):
                    bpy.data.objects.remove(obj, do_unlink=True)
            bpy.data.objects.remove(cube_obj, do_unlink=True)
        scene.slicing_cubes_collection.remove(idx)
        scene.slicing_cubes_index = min(idx, len(scene.slicing_cubes_collection)-1)
        self.report({'INFO'}, f"Removed slicing cube: {item.name}")
        return {'FINISHED'}

# -------------------------------------------------------------------
# (5) Calculate Rotations for AC Kinematics
# -------------------------------------------------------------------
class SLICINGCUBE_OT_slice(bpy.types.Operator):
    bl_idname = "slicingcube.calculate_rotations"
    bl_label = "Slice with AC Kinematics"

    def set_origin_to_a_axis(self, obj):
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        bpy.context.scene.cursor.location = (0, 0, bpy.context.scene.build_plate_distance)
        bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
        bpy.context.view_layer.update()

    def apply_visual_rotations(self, obj, c_rotation_rad, a_rotation_rad):
        pivot_point = mathutils.Vector((0.0, 0.0, bpy.context.scene.build_plate_distance))
        obj.location -= pivot_point
        rotation_c = mathutils.Matrix.Rotation(c_rotation_rad, 4, 'Z')
        rotation_a = mathutils.Matrix.Rotation(a_rotation_rad, 4, 'X')
        obj.matrix_world = rotation_a @ rotation_c @ obj.matrix_world
        obj.location += pivot_point

    def calculate_inverse_kinematics(self, context, tool_vector, tool_position):
        # --- PART 1: CALCULATE ROTATION ANGLES ---
        # This part calculates the required A and C angles to orient the tool correctly.
        Kx, Ky, Kz = tool_vector.x, tool_vector.y, tool_vector.z
        Kx, Ky, Kz = -Kx, -Ky, -Kz
        
        if Kz > 1.0: Kz = 1.0
        if Kz < -1.0: Kz = -1.0
        
        a_rad = acos(Kz)
        c_rad_kinematics = atan2(Kx, Ky)
        
        c_rad_output = c_rad_kinematics
        if context.scene.invert_c_axis:
            c_rad_output = -c_rad_kinematics

        # --- PART 2: CALCULATE WORK OFFSET (G10) ---
        # Duet's G10 L2 sets the origin of the new coordinate system (e.g., G55)
        # relative to the machine's base coordinates (when A=0, C=0).
        # We do NOT pre-rotate the coordinates. We simply pass the initial
        # position of the slicing cube relative to the machine's A-axis pivot.
        
        Qx, Qy, Qz = tool_position.x, tool_position.y, tool_position.z
        
        # The pivot point of the A-axis.
        pivot_z = context.scene.build_plate_distance
        
        # The work offset is the slicing cube's initial position.
        # We only need to make the Z-coordinate relative to the A-axis pivot.
        Px = Qx
        Py = Qy
        Pz = Qz - pivot_z

        return a_rad, c_rad_output, Px, Py, Pz

    def apply_boolean_intersect(self, original_mesh, slicing_cube):
        bpy.ops.object.select_all(action='DESELECT')
        original_mesh.select_set(True)
        bpy.context.view_layer.objects.active = original_mesh
        bool_intersect = original_mesh.modifiers.new(name="Intersect", type='BOOLEAN')
        bool_intersect.operation = 'INTERSECT'
        bool_intersect.object = slicing_cube
        bpy.ops.object.modifier_apply(modifier=bool_intersect.name)

    def apply_boolean_difference(self, selected_mesh, slicing_cube):
        if selected_mesh == slicing_cube: return
        bpy.context.view_layer.objects.active = selected_mesh
        bool_difference = selected_mesh.modifiers.new(name="Difference", type='BOOLEAN')
        bool_difference.operation = 'DIFFERENCE'
        bool_difference.object = slicing_cube
        bpy.ops.object.modifier_apply(modifier=bool_difference.name)

    def execute(self, context):
        scene = context.scene
        slicing_cubes = [bpy.data.objects[item.name] for item in scene.slicing_cubes_collection]
        selected_mesh = context.active_object
        if not selected_mesh or selected_mesh.type != 'MESH':
            self.report({'ERROR'}, "Please select a valid mesh object!")
            return {'CANCELLED'}
        scene.selected_mesh = selected_mesh
        
        original_mesh_for_subtracting = selected_mesh.copy()
        original_mesh_for_subtracting.data = selected_mesh.data.copy()
        scene.collection.objects.link(original_mesh_for_subtracting)

        for slicing_cube in slicing_cubes:
            cube_data = next((item for item in scene.slicing_cubes_collection if item.name == slicing_cube.name), None)
            if cube_data and cube_data.isProcessed:
                continue
            
            slicing_cube_origin_initial = slicing_cube.location.copy()
            slicing_cube_matrix_world = slicing_cube.matrix_world.copy()

            mesh_copy = original_mesh_for_subtracting.copy()
            mesh_copy.data = original_mesh_for_subtracting.data.copy()
            scene.collection.objects.link(mesh_copy)
            self.apply_boolean_intersect(mesh_copy, slicing_cube)

            bottom_face_normal = slicing_cube.data.polygons[4].normal
            normal_in_world_space = (slicing_cube_matrix_world.to_3x3() @ bottom_face_normal).normalized()
            
            a_rad, c_rad, p_x, p_y, p_z = self.calculate_inverse_kinematics(context, normal_in_world_space, slicing_cube_origin_initial)
            
            self.set_origin_to_a_axis(mesh_copy)
            self.apply_visual_rotations(mesh_copy, c_rad, a_rad)
            mesh_copy.location.x = 0
            mesh_copy.location.y = 0

            piece = scene.sliced_pieces_collection.add()
            piece.name = f"Sliced_Piece_{len(scene.sliced_pieces_collection)}"
            piece.geometry_object = mesh_copy
            piece.x_offset, piece.y_offset, piece.z_offset = p_x, p_y, p_z
            piece.c_rotation, piece.a_rotation = math.degrees(c_rad), math.degrees(a_rad)

            self.apply_boolean_difference(original_mesh_for_subtracting, slicing_cube)
            if cube_data:
                cube_data.isProcessed = True
        
        original_name = selected_mesh.name
        bpy.data.objects.remove(selected_mesh, do_unlink=True)
        original_mesh_for_subtracting.name = original_name
        scene.selected_mesh = original_mesh_for_subtracting

        self.report({'INFO'}, "Slicing completed successfully.")
        return {'FINISHED'}

# -------------------------------------------------------------------
# (6) G-code Generation
# -------------------------------------------------------------------
class SLICINGCUBE_OT_generate_gcode(bpy.types.Operator):
    bl_idname = "slicingcube.generate_gcode"
    bl_label = "Generate G-code (PrusaSlicer)"
    bl_description = "Generate G-code for the sliced model using PrusaSlicer"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        scene = context.scene
        main_mesh = scene.selected_mesh
        if not main_mesh:
             self.report({'ERROR'}, "No main mesh found to slice.")
             return {'CANCELLED'}

        stl_path = self.export_stl(main_mesh)
        if not stl_path:
            return {'CANCELLED'}
            
        output_gcode_path = os.path.join(os.path.dirname(stl_path), main_mesh.name + ".gcode")
        self.slice_with_prusa_slicer(stl_path, output_gcode_path, scene.prusa_slicer_ini_path)
        
        with open(output_gcode_path, 'r+') as f:
            content = f.read()
            f.seek(0, 0)
            header = (
                "; START of {}\n"
                "G1 E-10.0 F1800\n"
                "G1 Z115.0 F10000\n"
                "G1 A0.0 C0.0 F15000\n"
                "G54\n"
                "G1 Y0 F5000\n"
                "G1 X0 F5000\n"
                "G1 Z5.0 F4000\n"
                "G1 E10.0 F700\n"
                "G92 E0\n"
            ).format(
                os.path.basename(output_gcode_path)
            )
            f.write(header + content)
            
        gcode_files = [output_gcode_path]
        
        work_offsets = ["G55", "G56", "G57", "G58", "G59", "G59.1", "G59.2", "G59.3"]
        max_offsets = len(work_offsets)
        
        for idx, sliced_piece_item in enumerate(scene.sliced_pieces_collection):
            sliced_piece = sliced_piece_item.geometry_object
            if not sliced_piece:
                continue
            
            x, y, z = sliced_piece_item.x_offset, sliced_piece_item.y_offset, sliced_piece_item.z_offset
            
            stl_path_piece = self.export_stl(sliced_piece)
            if not stl_path_piece:
                continue
            output_gcode_piece = os.path.join(os.path.dirname(stl_path_piece), sliced_piece.name + ".gcode")
            self.slice_with_prusa_slicer(stl_path_piece, output_gcode_piece, scene.prusa_slicer_ini_path)
            
            offset_idx = idx % max_offsets
            p_code = offset_idx + 2
            gxx_code = work_offsets[offset_idx]
            
            with open(output_gcode_piece, 'r+') as f:
                content = f.read()
                f.seek(0, 0)
                header = (
                    "; START of {}\n"
                    "G1 E-10.0 F1800\n"
                    "G1 Z115.0 F10000\n"
                    "G1 A{:.3f} C{:.3f} F15000\n"
                    "G10 L2 P{} X{:.3f} Y{:.3f} Z{:.3f}\n"
                    "{}\n"
                    "G1 Y0 F5000\n"
                    "G1 X0 F5000\n"
                    "G1 Z5.0 F4000\n"
                    "G1 E10.0 F700\n"
                    "G92 E0\n"
                ).format(
                    os.path.basename(output_gcode_piece),
                    sliced_piece_item.a_rotation,
                    sliced_piece_item.c_rotation,
                    p_code, x, y, z,
                    gxx_code
                )
                f.write(header + content)
            gcode_files.append(output_gcode_piece)
            
        combined_gcode_path = os.path.join(os.path.dirname(stl_path), "combined_output.gcode")
        with open(combined_gcode_path, 'w') as combined_file:
            for gcode_file in gcode_files:
                with open(gcode_file, 'r') as f:
                    combined_file.write(f.read())
            combined_file.write("M2\n")
            
        self.report({'INFO'}, f"Combined G-code file created: {combined_gcode_path}")
        return {'FINISHED'}

    def export_stl(self, obj):
        if not obj:
            self.report({'ERROR'}, "Cannot export STL: Object not found.")
            return None
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        filename = f"{obj.name}.stl"
        
        output_path = bpy.path.abspath(bpy.context.scene.output_directory_path)
        if not output_path or not os.path.isdir(output_path):
            self.report({'ERROR'}, f"Output directory not set or invalid: {output_path}")
            return None
            
        stl_export_path = os.path.join(output_path, filename)
        
        bpy.ops.wm.stl_export(filepath=stl_export_path, check_existing=False, export_selected_objects=True)
        
        self.report({'INFO'}, f"STL exported to: {stl_export_path}")
        return stl_export_path

    def slice_with_prusa_slicer(self, input_stl, output_gcode, config_ini):
        prusa_slicer_path = bpy.path.abspath(bpy.context.scene.prusa_slicer_executable_path)
        if not os.path.exists(prusa_slicer_path):
            self.report({'ERROR'}, f"PrusaSlicer executable not found at: {prusa_slicer_path}")
            return None
        command = [prusa_slicer_path, "--export-gcode", "--load", config_ini, "-o", output_gcode, input_stl]
        try:
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            self.report({'INFO'}, f"PrusaSlicer finished for {os.path.basename(input_stl)}")
            self.clean_gcode(output_gcode, output_gcode, extruder_axis_name=bpy.context.scene.extruder_axis_name)
        except subprocess.CalledProcessError as e:
            self.report({'ERROR'}, f"PrusaSlicer failed for {os.path.basename(input_stl)}\nError: {e.stderr}")
            return None
        return output_gcode

    def clean_gcode(self, input_gcode, output_gcode, extruder_axis_name):
        with open(input_gcode, 'r') as infile:
            lines = infile.readlines()
        processed_lines = []
        for line in lines:
            if "E" in line:
                line = line.replace("E", extruder_axis_name)
            processed_lines.append(line)
        with open(output_gcode, 'w') as outfile:
            outfile.writelines(processed_lines)

# -------------------------------------------------------------------
# (6) Sliced Off Parts Data
# -------------------------------------------------------------------
class SlicedPieceItem(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="Name", default="Sliced Piece")
    c_rotation: bpy.props.FloatProperty(name="C Rotation (Z)", default=0.0)
    a_rotation: bpy.props.FloatProperty(name="A Rotation (X)", default=0.0)
    x_offset: bpy.props.FloatProperty(name="X Offset", default=0.0)
    y_offset: bpy.props.FloatProperty(name="Y Offset", default=0.0)
    z_offset: bpy.props.FloatProperty(name="Z Offset", default=0.0)
    geometry_object: bpy.props.PointerProperty(name="Geometry Object", type=bpy.types.Object)

class SLICEDPIECE_UL_items(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        layout.label(text=item.name, icon='MESH_CUBE')

# -------------------------------------------------------------------
# (7) Main Panel
# -------------------------------------------------------------------
class SLICINGCUBE_OT_reset_slicing(bpy.types.Operator):
    bl_idname = "slicingcube.reset_slicing"
    bl_label = "Reset Slicing Data"
    def execute(self, context):
        scene = context.scene
        if scene.selected_mesh:
             bpy.data.objects.remove(scene.selected_mesh, do_unlink=True)
        
        for item in scene.sliced_pieces_collection:
            obj = item.geometry_object
            if obj and obj.name in bpy.data.objects:
                bpy.data.objects.remove(obj)
        scene.sliced_pieces_collection.clear()
        
        for item in scene.slicing_cubes_collection:
            obj = bpy.data.objects.get(item.name)
            if obj:
                for child in list(obj.children):
                    bpy.data.objects.remove(child)
                bpy.data.objects.remove(obj)
        scene.slicing_cubes_collection.clear()
        
        self.report({'INFO'}, "Slicing data reset.")
        return {'FINISHED'}


class VIEW3D_PT_5AxisPrinterSetup(bpy.types.Panel):
    bl_label = "5-Axis Printer Slicer (AC)"
    bl_idname = "VIEW3D_PT_5axis_slicer_ac"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "5-Axis Printer"
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        box = layout.box()
        box.label(text="Machine Settings")
        col = box.column(align=True)
        col.prop(scene, "build_plate_distance")
        col.prop(scene, "invert_c_axis")
        col.prop(scene, "extruder_axis_name")
        layout.separator()
        
        layout.label(text="Slicing Cubes")
        row = layout.row()
        row.template_list("SLICINGCUBE_UL_items", "", scene, "slicing_cubes_collection", scene, "slicing_cubes_index", rows=3)
        col = row.column(align=True)
        col.operator("slicingcube.add", icon='ADD', text="")
        col.operator("slicingcube.remove", icon='REMOVE', text="")
        
        if scene.slicing_cubes_index >= 0 and len(scene.slicing_cubes_collection) > 0:
            item = scene.slicing_cubes_collection[scene.slicing_cubes_index]
            obj = bpy.data.objects.get(item.name)
            if obj:
                box = layout.box()
                box.label(text=f"Transform: {obj.name}")
                col = box.column(align=True)
                col.prop(obj, "location", text="Location")
                col.prop(obj, "rotation_euler", text="Rotation")
                col.prop(obj, "scale", text="Scale")
        
        layout.operator("slicingcube.calculate_rotations", text="Calculate and Slice", icon="DRIVER_ROTATIONAL_DIFFERENCE")
        layout.separator()
        layout.label(text="Sliced Pieces")
        row = layout.row()
        row.template_list("SLICEDPIECE_UL_items", "", scene, "sliced_pieces_collection", scene, "sliced_pieces_index", rows=3)
        
        if scene.sliced_pieces_index >= 0 and len(scene.sliced_pieces_collection) > 0:
            piece = scene.sliced_pieces_collection[scene.sliced_pieces_index]
            box = layout.box()
            col = box.column(align=True)
            col.prop(piece, "a_rotation", text="A Axis")
            col.prop(piece, "c_rotation", text="C Axis")
            col.prop(piece, "x_offset", text="X Offset")
            col.prop(piece, "y_offset", text="Y Offset")
            col.prop(piece, "z_offset", text="Z Offset")

        layout.separator()
        layout.label(text="PrusaSlicer Settings")
        layout.prop(scene, "prusa_slicer_executable_path")
        layout.prop(scene, "prusa_slicer_ini_path")
        layout.prop(scene, "output_directory_path")
        layout.operator("slicingcube.generate_gcode", text="Generate G-code", icon="FILE_SCRIPT")
        layout.separator()
        layout.operator("slicingcube.reset_slicing", text="Reset Slicing Data", icon="PANEL_CLOSE")

# -------------------------------------------------------------------
# Registration
# -------------------------------------------------------------------
classes = (
    SlicingCubeItem, SLICINGCUBE_UL_items, SLICINGCUBE_OT_add, SLICINGCUBE_OT_remove,
    SLICINGCUBE_OT_slice, SLICINGCUBE_OT_generate_gcode, VIEW3D_PT_5AxisPrinterSetup,
    SlicedPieceItem, SLICEDPIECE_UL_items,
    SLICINGCUBE_OT_reset_slicing
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.slicing_cubes_collection = bpy.props.CollectionProperty(type=SlicingCubeItem)
    bpy.types.Scene.slicing_cubes_index = bpy.props.IntProperty(default=-1)
    bpy.types.Scene.sliced_pieces_collection = CollectionProperty(type=SlicedPieceItem)
    bpy.types.Scene.sliced_pieces_index = bpy.props.IntProperty(default=0)
    bpy.types.Scene.build_plate_distance = bpy.props.FloatProperty(name="A-Axis Distance", default=0.0, unit='LENGTH', description="Distance between the world origin and the A-axis pivot point.")
    bpy.types.Scene.selected_mesh = bpy.props.PointerProperty(name="Selected Mesh", type=bpy.types.Object, description="The mesh selected for slicing")
    bpy.types.Scene.extruder_axis_name = bpy.props.StringProperty(name="Extruder Letter", default="E", description="The letter for the extruder axis.")
    bpy.types.Scene.prusa_slicer_executable_path = bpy.props.StringProperty(name="PrusaSlicer Executable", default="", subtype='FILE_PATH', description="Path to the prusa-slicer-console executable.")
    bpy.types.Scene.prusa_slicer_ini_path = bpy.props.StringProperty(name="PrusaSlicer INI Config", default="", subtype='FILE_PATH', description="Path to your PrusaSlicer .ini configuration file.")
    bpy.types.Scene.output_directory_path = bpy.props.StringProperty(name="Output Directory", default="", subtype='DIR_PATH', description="Directory to save generated STL and G-code files.")
    
    bpy.types.Scene.invert_c_axis = BoolProperty(
        name="Invert C-Axis Rotation (CW+)",
        description="Invert C-axis for clockwise positive rotation (e.g., for Z-down machine coordinates)",
        default=True
    )

def unregister():
    prop_names = (
        'slicing_cubes_collection', 'slicing_cubes_index', 'sliced_pieces_collection', 
        'sliced_pieces_index', 'build_plate_distance', 'selected_mesh', 'extruder_axis_name', 
        'prusa_slicer_executable_path', 'prusa_slicer_ini_path', 'output_directory_path',
        'invert_c_axis'
    )
    for prop_name in prop_names:
        if hasattr(bpy.types.Scene, prop_name):
            delattr(bpy.types.Scene, prop_name)
            
    for cls in reversed(classes):
        if hasattr(bpy.utils, "unregister_class"):
            try:
                bpy.utils.unregister_class(cls)
            except RuntimeError:
                pass

if __name__ == "__main__":
    register()