# autocut_gcode_plugin/gcode_writer.py
import bpy
import mathutils
import math

class GCodeExporter:
    def __init__(self, all_toolpath_data, slice_settings, scene): # Changed scene_settings to scene for context
        self.all_toolpath_data = all_toolpath_data # List of dicts, each for an orientation
        self.slice_settings = slice_settings     # SlicingSettings PropertyGroup instance
        self.scene = scene                       # Blender scene, for things like unit_settings or active_object if needed
        self.gcode_lines = []
        self.current_extruder_pos = 0.0          # Tracks absolute extruder E position

    def _add_line(self, line, comment=""):
        full_line = line.strip()
        if comment:
            full_line += f" ; {comment}"
        self.gcode_lines.append(full_line)

    def _calculate_extrusion(self, segment_length_mm, layer_height_mm, extrusion_width_mm):
        if self.slice_settings.filament_diameter <= 1e-6: # Avoid division by zero
            print("Error: Filament diameter is zero or too small.")
            return 0.0

        filament_radius_mm = self.slice_settings.filament_diameter / 2.0
        filament_area_mm2 = math.pi * (filament_radius_mm ** 2)

        volume_to_extrude_mm3 = segment_length_mm * layer_height_mm * extrusion_width_mm
        length_of_filament_mm = volume_to_extrude_mm3 / filament_area_mm2

        return length_of_filament_mm * self.slice_settings.extrusion_multiplier

    def _get_current_tool_e_pos_from_script(self, script_lines_string, initial_e_pos):
        """Rudimentary parsing of G1 E moves in a script to update E position."""
        new_e_pos = initial_e_pos
        for line in script_lines_string.splitlines():
            line_upper = line.upper()
            if "G92 E" in line_upper:
                try:
                    new_e_pos = float(line_upper.split("E")[1].split()[0])
                except ValueError:
                    print(f"Warning: Could not parse E value from G92 in script: {line}")
            elif "G1 E" in line_upper: # Assumes G1 E is absolute if M82, or relative if M83
                                      # Given header usually has M82, this should be absolute target.
                                      # This is simplified: for M83 this logic would be wrong.
                                      # For this plugin, we assume M82 (absolute E) is set in header.
                try:
                    # This should be the target E value if M82
                    new_e_pos = float(line_upper.split("E")[1].split()[0])
                except ValueError:
                    print(f"Warning: Could not parse E value from G1 in script: {line}")
        return new_e_pos


    def export_gcode(self, filepath):
        self.gcode_lines = []
        self.current_extruder_pos = 0.0 # Reset for each export

        # 1. Header
        header_script = self.slice_settings.gcode_header.replace('\\n', '\n') # Allow \n in prop string
        for line in header_script.splitlines():
            self._add_line(line)
        # Update E pos based on header (e.g. if it has G92 E0)
        self.current_extruder_pos = self._get_current_tool_e_pos_from_script(header_script, self.current_extruder_pos)


        # Determine a general safe Z height based on the active object, if possible
        # This safe_z is in WORLD coordinates.
        safe_z_absolute_world = 10.0 # Default if no object
        if bpy.context.active_object and bpy.context.active_object.type == 'MESH':
            obj = bpy.context.active_object
            if obj.data.vertices:
                 # Calculate max Z in world space of the original object
                max_obj_z_world = max((obj.matrix_world @ v.co).z for v in obj.data.vertices)
                safe_z_absolute_world = max_obj_z_world + (self.slice_settings.z_hop_height if self.slice_settings.z_hop_height > 0 else 5.0) # Use z_hop or default
            else: # object has no vertices
                safe_z_absolute_world = obj.matrix_world.translation.z + 10.0
        else: # No active mesh object, use a sensible default
            safe_z_absolute_world = 20.0


        # 2. Iterate through each orientation setup
        for setup_idx, setup_data in enumerate(self.all_toolpath_data):
            self._add_line(f"
; --- Setup {setup_idx + 1} of {len(self.all_toolpath_data)} ---") # Add total setups for context

            wcs_p_num = setup_data['wcs_p_number']
            wcs_origin_world = mathutils.Vector(setup_data['wcs_origin_xyz']) # Stored as list/tuple, convert to Vector
            A_deg, C_deg = setup_data['wcs_rotation_ac_deg']
            ax_A, ax_C = setup_data['wcs_gcode_axes'] # Tuple of axis letters

            slicing_frame_to_wcs_transform = mathutils.Matrix(setup_data['slicing_frame_to_world_transform']) # Convert from list of lists if needed

            # Before Orientation Change G-code
            # Note: {safe_z} in the script should refer to a Z in the CURRENT WCS before rotation.
            # This is tricky. For now, let's assume {safe_z} is an absolute Z in machine base coordinates.
            # This requires careful machine setup or a more sophisticated safe_z calculation.
            # For simplicity, we use safe_z_absolute_world calculated once.
            script_vars = {
                "safe_z": f"{safe_z_absolute_world:.4f}",
                "travel_feedrate": f"{self.slice_settings.feed_rate_travel:.0f}",
                "retraction_amount": f"{self.slice_settings.retraction_amount:.4f}",
                "retraction_speed": f"{self.slice_settings.retraction_speed:.0f}"
            }
            before_script = self.slice_settings.gcode_before_orientation_change.format(**script_vars)
            before_script = before_script.replace('\\n', '\n')
            for line in before_script.splitlines(): self._add_line(line)
            self.current_extruder_pos = self._get_current_tool_e_pos_from_script(before_script, self.current_extruder_pos)

            # Rotate A and C axes (absolute coordinates)
            self._add_line(f"G0 {ax_A}{A_deg:.4f} {ax_C}{C_deg:.4f} F{self.slice_settings.feed_rate_rotation:.0f}", f"Rotate to A={A_deg:.3f}, C={C_deg:.3f}")

            # Set WCS (G10 L2 Pn X Y Z A C) - some controllers might also want A C in G10.
            # Standard Fanuc: G10 L2 Px X Y Z (A C are set by G0/G1 moves typically)
            self._add_line(f"G10 L2 P{wcs_p_num} X{wcs_origin_world.x:.4f} Y{wcs_origin_world.y:.4f} Z{wcs_origin_world.z:.4f}", f"Set WCS P{wcs_p_num} origin")

            if 1 <= wcs_p_num <= 6: # G54-G59
                 g_wcs = 53 + wcs_p_num
                 self._add_line(f"G{g_wcs}", f"Activate WCS G{g_wcs} (P{wcs_p_num})")
            elif wcs_p_num > 6: # G54.1 Px for extended WCS
                 self._add_line(f"G54.1 P{wcs_p_num - 6}", f"Activate Extended WCS P{wcs_p_num-6} (Original P{wcs_p_num})")
            else: # Should not happen with P_num >= 1
                self._add_line(f"; WARNING: WCS P-number {wcs_p_num} is unusual.", "")

            # After Orientation Change G-code
            after_script = self.slice_settings.gcode_after_orientation_change.format(**script_vars) # Same vars can be used
            after_script = after_script.replace('\\n', '\n')
            for line in after_script.splitlines(): self._add_line(line)
            self.current_extruder_pos = self._get_current_tool_e_pos_from_script(after_script, self.current_extruder_pos)

            # --- Process layers for this orientation ---
            last_pos_wcs = None # Track last position for Z hop logic

            for layer_idx, layer_data in enumerate(setup_data['layers']):
                layer_z_slice_frame = layer_data['z']
                self._add_line(f"; Layer {layer_idx + 1} at Z_slice_frame={layer_z_slice_frame:.4f}")

                for path_idx, path_points_slice_frame in enumerate(layer_data['paths']):
                    if not path_points_slice_frame or len(path_points_slice_frame) < 1: # Allow single point for Z move if needed
                        continue

                    # First point of the path (travel move)
                    # Path points are 2D (x,y) in slice frame. Add z_slice_frame for 3D.
                    p0_slice_3d = mathutils.Vector((path_points_slice_frame[0].x, path_points_slice_frame[0].y, layer_z_slice_frame))
                    p0_wcs = slicing_frame_to_wcs_transform @ p0_slice_3d # Transform to current WCS

                    # Z-Hop before travel if enabled and if there was a previous position
                    if self.slice_settings.z_hop_height > 0 and last_pos_wcs is not None:
                        self._add_line(f"G1 Z{last_pos_wcs.z + self.slice_settings.z_hop_height:.4f} F{self.slice_settings.feed_rate_travel:.0f}", "Z Hop up")

                    # Travel to the start of the path (X, Y, Z)
                    self._add_line(f"G0 X{p0_wcs.x:.4f} Y{p0_wcs.y:.4f} Z{p0_wcs.z:.4f}", f"Travel to start of path {path_idx+1}")
                    last_pos_wcs = p0_wcs.copy() # Update last position

                    # Lower Z after Z-Hop (if hopped)
                    if self.slice_settings.z_hop_height > 0 and path_idx > 0: # Check path_idx to avoid Z-hop down on first path of layer
                         self._add_line(f"G1 Z{p0_wcs.z:.4f} F{self.slice_settings.feed_rate_travel:.0f}", "Z Hop down")

                    # Prime after travel if needed (usually handled by after_orientation_change or specific prime towers)
                    # For simplicity, let's assume priming is part of after_orientation_change for now.

                    # Extrude along the path (if more than one point)
                    if len(path_points_slice_frame) > 1:
                        for i in range(len(path_points_slice_frame) - 1):
                            # Current point is p0_wcs (or last_pos_wcs)
                            p_start_wcs = last_pos_wcs

                            # Next point
                            p_end_slice_3d = mathutils.Vector((path_points_slice_frame[i+1].x, path_points_slice_frame[i+1].y, layer_z_slice_frame))
                            p_end_wcs = slicing_frame_to_wcs_transform @ p_end_slice_3d

                            segment_length_mm = (p_end_wcs - p_start_wcs).length
                            extrusion_for_segment = self._calculate_extrusion(
                                segment_length_mm,
                                self.slice_settings.layer_height,
                                self.slice_settings.nozzle_diameter # Assuming extrusion width = nozzle diameter
                            )
                            new_e_target = self.current_extruder_pos + extrusion_for_segment

                            self._add_line(f"G1 X{p_end_wcs.x:.4f} Y{p_end_wcs.y:.4f} Z{p_end_wcs.z:.4f} E{new_e_target:.5f} F{self.slice_settings.feed_rate_print:.0f}")
                            self.current_extruder_pos = new_e_target
                            last_pos_wcs = p_end_wcs.copy()
                    else: # Single point path, just ensure we are at the Z
                         self._add_line(f"G1 Z{p0_wcs.z:.4f} F{self.slice_settings.feed_rate_print:.0f}", "Set Z for single point path")


            # Retraction at the end of all paths for this orientation (if not the last setup)
            # This is now handled by the gcode_before_orientation_change script of the NEXT setup, or footer.
            # if setup_idx < len(self.all_toolpath_data) - 1:
            #    # Retract logic here, or ensure it's in 'before_orientation_change'
            #    pass

        # 3. Footer
        footer_script = self.slice_settings.gcode_footer.replace('\\n', '\n')
        # Update E based on footer (e.g. if it has G92 E0 or retractions)
        # self.current_extruder_pos = self._get_current_tool_e_pos_from_script(footer_script, self.current_extruder_pos) # Not strictly needed as E is absolute
        for line in footer_script.splitlines():
            self._add_line(line)

        # Write to file
        try:
            with open(filepath, 'w') as f:
                for line_item in self.gcode_lines:
                    f.write(line_item + '\n')
            print(f"G-code successfully written to {filepath}")
            return True
        except IOError as e:
            print(f"Error writing G-code file: {e}")
            self.gcode_lines = [f"; ERROR: Could not write file: {e}"] # Make error visible in file if it's somehow created
            return False
