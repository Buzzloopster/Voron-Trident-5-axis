# autocut_gcode_plugin/slicer.py
import bpy
import bmesh
import mathutils
from mathutils import Vector

def get_mesh_bounding_box_bm(bm, matrix_world):
    """Calculates the world-space bounding box of a BMesh after applying a matrix."""
    if not bm.verts:
        return Vector((0,0,0)), Vector((0,0,0))

    min_coord = Vector((float('inf'), float('inf'), float('inf')))
    max_coord = Vector((float('-inf'), float('-inf'), float('-inf')))

    for vert in bm.verts:
        world_co = matrix_world @ vert.co
        min_coord.x = min(min_coord.x, world_co.x)
        min_coord.y = min(min_coord.y, world_co.y)
        min_coord.z = min(min_coord.z, world_co.z)
        max_coord.x = max(max_coord.x, world_co.x)
        max_coord.y = max(max_coord.y, world_co.y)
        max_coord.z = max(max_coord.z, world_co.z)
    return min_coord, max_coord


def slice_mesh_at_z(obj_to_slice_locally, z_height_local):
    """
    Slices a mesh at a given Z height IN ITS LOCAL COORDINATE SYSTEM.
    Assumes obj_to_slice_locally is already oriented as desired for Z-up slicing.
    Returns a list of 2D contours (list of lists of Vector((x,y))).
    """
    if obj_to_slice_locally is None or obj_to_slice_locally.type != 'MESH':
        print("Slicer (slice_mesh_at_z): Invalid object provided.")
        return []

    current_mode = bpy.context.mode
    if current_mode == 'EDIT_MESH':
        bpy.ops.object.mode_set(mode='OBJECT') # BMesh ops work best from object mode with fresh data

    bm = bmesh.new()
    bm.from_mesh(obj_to_slice_locally.data) # Use object's local data
    bm.transform(obj_to_slice_locally.matrix_world) # Bring to world space if needed by caller, BUT
                                                # this function assumes it's ALREADY oriented.
                                                # For this function, we assume obj_to_slice_locally IS the Z-up representation.
                                                # So, we operate purely on its local data.
                                                # Let's re-fetch if the above transform was a mistake for this func's contract.
    bm.free() # Free previous bm
    bm = bmesh.new()
    bm.from_mesh(obj_to_slice_locally.data)


    # Define the cutting plane in the object's local space
    plane_co = Vector((0, 0, z_height_local))
    plane_no = Vector((0, 0, 1)) # Slicing along local Z axis

    try:
        geom_dict = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:],
                                           plane_co=plane_co, plane_no=plane_no,
                                           clear_inner=False, clear_outer=False,
                                           use_snap_center=False) # Added use_snap_center
    except Exception as e:
        print(f"Slicer (slice_mesh_at_z): BMesh bisect_plane failed: {e}")
        bm.free()
        if current_mode == 'EDIT_MESH': bpy.ops.object.mode_set(mode='EDIT')
        return []

    cut_edges = [e for e in geom_dict.get('geom_cut', []) if isinstance(e, bmesh.types.BMEdge)]

    if not cut_edges:
        bm.free()
        if current_mode == 'EDIT_MESH': bpy.ops.object.mode_set(mode='EDIT')
        return []

    # --- Connect edges to form loops (contours) ---
    # This is a simplified graph traversal. It may not handle all complex cases robustly.
    contours = []

    # Create a copy of cut_edges to iterate over, as we modify it
    edges_to_process = list(cut_edges)

    while edges_to_process:
        current_loop_verts_co = []

        # Start a new loop from an arbitrary edge
        start_edge = edges_to_process.pop(0)

        v_current = start_edge.verts[0]
        v_start_loop = v_current
        current_loop_verts_co.append(v_current.co.xy.copy()) # Store 2D coordinate

        # Follow the loop
        # Max iterations to prevent infinite loops on malformed geometry
        max_iter = len(cut_edges) * 2
        iter_count = 0

        edge_runner = start_edge

        while iter_count < max_iter:
            iter_count += 1
            # Find the other vertex of the current edge
            v_next_on_edge = edge_runner.other_vert(v_current)

            if v_next_on_edge == v_start_loop: # Loop closed
                break

            current_loop_verts_co.append(v_next_on_edge.co.xy.copy())
            v_current = v_next_on_edge # Move to the next vertex

            # Find the next edge connected to v_current, that is not edge_runner
            found_next_edge_in_loop = None
            for next_candidate_edge in v_current.link_edges:
                if next_candidate_edge in cut_edges and next_candidate_edge is not edge_runner:
                    found_next_edge_in_loop = next_candidate_edge
                    break

            if found_next_edge_in_loop:
                edge_runner = found_next_edge_in_loop
                if edge_runner in edges_to_process:
                    edges_to_process.remove(edge_runner) # Mark as used for this loop
            else: # Dead end - loop didn't close or it's an open segment
                break

        if len(current_loop_verts_co) > 1: # Need at least 2 points
            contours.append([Vector((v.x, v.y)) for v in current_loop_verts_co])
        elif iter_count >= max_iter:
            print("Slicer (slice_mesh_at_z): Max iterations reached while forming a contour. Contour might be malformed.")


    bm.free()
    if current_mode == 'EDIT_MESH': bpy.ops.object.mode_set(mode='EDIT')
    return contours


def generate_toolpaths_for_orientation(obj_to_slice, world_to_slicing_frame_transform, slice_settings, context):
    """
    Generates all toolpaths for a given object and a specific slicing orientation.
    obj_to_slice: The Blender object to slice.
    world_to_slicing_frame_transform: Matrix that transforms obj_to_slice from world to be Z-up for slicing.
    slice_settings: A SlicingSettings PropertyGroup instance.
    Returns: A list of layers, where each layer is {'z': z_height_in_slice_frame, 'paths': [toolpaths (list of Vector((x,y)))]}.
    """
    all_layers_toolpaths = []

    if not obj_to_slice or obj_to_slice.type != 'MESH':
        print("Slicer (generate_toolpaths): No valid mesh object provided.")
        return all_layers_toolpaths

    # 1. Create a temporary transformed copy of the mesh for slicing
    # This avoids modifying the original object and simplifies Z-up slicing.

    # Ensure we're in object mode for certain operations if needed
    current_mode_bpy = bpy.context.mode
    if current_mode_bpy != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')

    # Duplicate the object and its data to avoid altering the original
    # This is safer practice for tools.
    temp_obj = obj_to_slice.copy()
    temp_obj.data = obj_to_slice.data.copy() # Duplicate mesh data as well
    temp_obj.name = "temp_slicing_object"
    context.collection.objects.link(temp_obj) # Link to scene to make it usable by ops if needed, and for bmesh.from_object

    # Apply the transformation to the temporary object itself
    # This makes its local Z-axis the slicing direction.
    temp_obj.matrix_world = world_to_slicing_frame_transform @ obj_to_slice.matrix_world
    # After this, temp_obj is now in the "slicing frame". Its local Z is the slice direction.
    # Its local origin is where the world origin was after transformation.

    # We need the bounding box of this temp_obj in its *local* coordinates,
    # as slice_mesh_at_z will operate on its local Z.
    bm_eval = bmesh.new()
    bm_eval.from_mesh(temp_obj.data) # Get local coordinates
    # No matrix_world multiplication here as we want local bounds of the already transformed object

    if not bm_eval.verts:
        bm_eval.free()
        bpy.data.objects.remove(temp_obj, do_unlink=True) # also removes mesh data if only user
        print("Slicer (generate_toolpaths): Temporary mesh for slicing is empty.")
        if current_mode_bpy != 'OBJECT': bpy.ops.object.mode_set(mode=current_mode_bpy)
        return all_layers_toolpaths

    # Calculate bounding box in the local coordinate system of temp_obj
    min_z_local = min(v.co.z for v in bm_eval.verts)
    max_z_local = max(v.co.z for v in bm_eval.verts)
    bm_eval.free()

    layer_height = slice_settings.layer_height
    if layer_height <= 0:
        print("Slicer (generate_toolpaths): Invalid layer height.")
        bpy.data.objects.remove(temp_obj, do_unlink=True)
        if current_mode_bpy != 'OBJECT': bpy.ops.object.mode_set(mode=current_mode_bpy)
        return all_layers_toolpaths

    # Start slicing from the middle of the first layer
    current_z_local = min_z_local + layer_height / 2.0

    print(f"Slicer (generate_toolpaths): Slicing '{temp_obj.name}' from Z={min_z_local:.3f} to Z={max_z_local:.3f}, Layer Height: {layer_height:.3f}")

    while current_z_local < max_z_local:
        # slice_mesh_at_z takes the object and a Z height in that object's local space.
        # temp_obj is already oriented correctly.
        contours_at_z_local = slice_mesh_at_z(temp_obj, current_z_local)

        if contours_at_z_local:
            layer_toolpaths = []
            for contour_local_xy in contours_at_z_local: # These are List[Vector(x,y)]
                if len(contour_local_xy) > 1:
                    # For now, these are raw 2D contours in the slicing frame's XY plane.
                    # TODO: Implement wall generation (offsetting)
                    # TODO: Implement infill generation
                    layer_toolpaths.append(contour_local_xy)

            if layer_toolpaths:
                # Store Z height in the slicing frame's coordinate system
                all_layers_toolpaths.append({'z': current_z_local, 'paths': layer_toolpaths})

        current_z_local += layer_height

    # Clean up the temporary object
    bpy.data.objects.remove(temp_obj, do_unlink=True)
    # The temp_obj.data (mesh data) should also be removed if it has no other users.
    # bpy.data.meshes.remove(temp_mesh_data) # This was handled by removing temp_obj if it was the only user.

    if current_mode_bpy != 'OBJECT': # Restore original mode
        bpy.ops.object.mode_set(mode=current_mode_bpy)

    print(f"Slicer (generate_toolpaths): Generated {len(all_layers_toolpaths)} layers for this orientation.")
    return all_layers_toolpaths
