# autocut_gcode_plugin/gcode_writer.py
import bpy
import mathutils
import math

class GCodeExporter:
    def __init__(self, all_toolpath_data, slice_settings, scene_settings):
        self.all_toolpath_data = all_toolpath_data
        self.slice_settings = slice_settings
        self.scene_settings = scene_settings
        self.gcode_lines = []
        self.current_extruder_pos = 0.0

    def _add_line(self, line_content, comment_text=""):
        line_str = str(line_content)
        if comment_text:
            comment_str = str(comment_text).replace(";", ",")
            self.gcode_lines.append(f"{line_str} ; {comment_str}")
        else:
            self.gcode_lines.append(line_str)

    def _calculate_extrusion(self, segment_length):
        if self.slice_settings.filament_diameter <= 1e-6:
            return 0.0
        filament_radius = self.slice_settings.filament_diameter / 2.0
        filament_area = math.pi * (filament_radius ** 2)
        extrusion_width = self.slice_settings.nozzle_diameter
        layer_height = self.slice_settings.layer_height

        if extrusion_width <= 1e-6 or layer_height <= 1e-6:
            return 0.0

        volume_to_extrude = segment_length * layer_height * extrusion_width
        if filament_area > 1e-6:
            return (volume_to_extrude / filament_area) * self.slice_settings.extrusion_multiplier
        return 0.0

    def _is_numeric_val(self, val_str):
        if not isinstance(val_str, str):
            return False
        temp_str = val_str.lstrip("+-")
        return temp_str.replace('.', '', 1).isdigit()

    def _update_e_pos_from_script_line(self, gcode_line_text):
        gcode_line = str(gcode_line_text).upper()

        parts = gcode_line.split()
        for i, part in enumerate(parts):
            if part.startswith("E"):
                val_str = part[1:]
                if val_str and self._is_numeric_val(val_str):
                    self.current_extruder_pos = float(val_str)
                    break

    def export_gcode(self, filepath):
        self.gcode_lines = []
        self.current_extruder_pos = 0.0

        # Header
        header_setting = str(self.slice_settings.gcode_header)
        for header_line in header_setting.splitlines():
            self._add_line(header_line)
            self._update_e_pos_from_script_line(header_line)

        # Get safe Z height
        max_obj_z = 0.0
        active_obj = bpy.context.active_object
        if active_obj and active_obj.type == 'MESH' and active_obj.data.vertices:
            max_obj_z = max((active_obj.matrix_world @ v.co).z for v in active_obj.data.vertices)

        safe_z_absolute = max_obj_z + 10.0

        for setup_idx, setup_data in enumerate(self.all_toolpath_data):
            self._add_line(f"; --- Setup {setup_idx + 1} ---")

            wcs_p_num = int(setup_data['wcs_p_number'])
            wcs_origin = mathutils.Vector(setup_data['wcs_origin_xyz'])
            A_deg, C_deg = map(float, setup_data['wcs_rotation_ac_deg'])
            ax_A = str(self.slice_settings.gcode_axis_A_letter)
            ax_C = str(self.slice_settings.gcode_axis_C_letter)
            slicing_frame_to_wcs_transform = mathutils.Matrix(setup_data['slicing_frame_to_world_transform'])

            format_dict = {
                "safe_z": f"{safe_z_absolute:.3f}",
                "travel_feedrate": f"{self.slice_settings.feed_rate_travel:.0f}"
            }

            # Before orientation change script
            try:
                before_script = self.slice_settings.gcode_before_orientation_change.format(**format_dict)
            except KeyError:
                before_script = self.slice_settings.gcode_before_orientation_change

            for line in before_script.splitlines():
                self._add_line(line)
                self._update_e_pos_from_script_line(line)

            # Orientation and WCS commands
            self._add_line(f"G0 {ax_A}{A_deg:.3f} {ax_C}{C_deg:.3f} F{self.slice_settings.feed_rate_rotation:.0f}",
                           f"Rotate to A={A_deg:.2f}, C={C_deg:.2f}")

            self._add_line(f"G10 L2 P{wcs_p_num} X{wcs_origin.x:.4f} Y{wcs_origin.y:.4f} Z{wcs_origin.z:.4f}",
                           f"Set WCS P{wcs_p_num}")

            # Activate WCS
            if 1 <= wcs_p_num <= 6:
                self._add_line(f"G{53 + wcs_p_num}", f"Activate WCS G{53 + wcs_p_num} (P{wcs_p_num})")
            elif 7 <= wcs_p_num <= 260:
                p_sub = wcs_p_num - 6
                self._add_line(f"G54.1 P{p_sub}", f"Activate WCS G54.1 P{p_sub} (Original G10 P{wcs_p_num})")
            else:
                self._add_line(f"; WARNING: WCS P-number {wcs_p_num} cannot be mapped", "")

            # After orientation change script
            try:
                after_script = self.slice_settings.gcode_after_orientation_change.format(**format_dict)
            except KeyError:
                after_script = self.slice_settings.gcode_after_orientation_change

            for line in after_script.splitlines():
                self._add_line(line)
                self._update_e_pos_from_script_line(line)

            # Layer and path generation
            for layer_idx, layer_data in enumerate(setup_data['layers']):
                z = float(layer_data['z'])
                self._add_line(f"; Layer {layer_idx + 1}, Z_slice_frame={z:.4f}")

                last_pos_wcs = None
                for path_idx, path_points in enumerate(layer_data['paths']):
                    if not path_points or len(path_points) < 2:
                        continue

                    p0 = mathutils.Vector((float(path_points[0][0]), float(path_points[0][1]), z))
                    p0_wcs = slicing_frame_to_wcs_transform @ p0

                    hop_z = last_pos_wcs.z if last_pos_wcs else p0_wcs.z
                    if self.slice_settings.z_hop_height > 1e-6 and last_pos_wcs:
                        self._add_line(f"G1 Z{hop_z + self.slice_settings.z_hop_height:.4f} F{self.slice_settings.feed_rate_travel:.0f}",
                                       "Z Hop up")

                    self._add_line(f"G0 X{p0_wcs.x:.4f} Y{p0_wcs.y:.4f} Z{p0_wcs.z:.4f} F{self.slice_settings.feed_rate_travel:.0f}",
                                   f"Travel to start of path {path_idx + 1}")

                    if self.slice_settings.z_hop_height > 1e-6 and last_pos_wcs:
                        self._add_line(f"G1 Z{p0_wcs.z:.4f} F{self.slice_settings.feed_rate_travel:.0f}", "Z Hop down")

                    for i in range(len(path_points) - 1):
                        p_start = mathutils.Vector((float(path_points[i][0]), float(path_points[i][1]), z))
                        p_end = mathutils.Vector((float(path_points[i + 1][0]), float(path_points[i + 1][1]), z))

                        p_start_wcs = slicing_frame_to_wcs_transform @ p_start
                        p_end_wcs = slicing_frame_to_wcs_transform @ p_end

                        segment_length = (p_end_wcs - p_start_wcs).length
                        extrusion = self._calculate_extrusion(segment_length)
                        self.current_extruder_pos += extrusion

                        self._add_line(f"G1 X{p_end_wcs.x:.4f} Y{p_end_wcs.y:.4f} Z{p_end_wcs.z:.4f} E{self.current_extruder_pos:.5f} F{self.slice_settings.feed_rate_print:.0f}")

                    last_pos = mathutils.Vector((float(path_points[-1][0]), float(path_points[-1][1]), z))
                    last_pos_wcs = slicing_frame_to_wcs_transform @ last_pos

        # Footer
        footer_setting = str(self.slice_settings.gcode_footer)
        for footer_line in footer_setting.splitlines():
            self._add_line(footer_line)
            self._update_e_pos_from_script_line(footer_line)

        try:
            with open(filepath, 'w') as f_out:
                for output_gcode_line in self.gcode_lines:
                    f_out.write(str(output_gcode_line) + '\n')
            return True
        except IOError as e:
            print(f"Error writing G-code to {filepath}: {e}")
            return False
        except Exception as e_gen:
            print(f"General error during G-code file write: {e_gen}")
            return False