# AutoCut GCode for 3+2 Axis Blender Plugin

**Version:** 0.2.0
**Author:** AI Assistant (Jules)

## Description

This Blender plugin facilitates 3+2 axis G-code generation for Fused Deposition Modeling (FDM) 3D printing. It allows users to select planar faces on a 3D model, which are then used to perform Boolean cutting operations. For each resulting orientation, the plugin calculates appropriate Work Coordinate System (WCS) offsets and A/C rotation angles, slices the model (generating basic contours), and exports G-code suitable for 3+2 axis machining/printing.

## Features

*   **Planar Face Selection:** Define cutting planes and subsequent machining orientations by selecting planar faces in Edit Mode.
*   **Boolean Cutting:** Automatically performs Boolean difference operations on the model using the selected planes.
*   **WCS Calculation:** Calculates WCS origins (X,Y,Z) and A/C rotation angles for each distinct orientation.
*   **3+2 Axis G-Code Generation:** Outputs G-code with commands for axis rotations (A, C) and WCS changes (G10 L2 Pn, G5x).
*   **Basic Slicing:** Generates 2D contours (toolpaths) for each layer within each orientation.
*   **Customizable G-Code:** Allows user-defined G-code snippets for headers, footers, and scripts before/after orientation changes.
*   **Configurable Parameters:** UI panel for adjusting slicing settings (layer height, nozzle diameter), G-code parameters (feed rates, WCS numbering, axis letters), and extrusion settings.

## Installation

1.  Download the plugin ZIP file (ensure the `autocut_gcode_plugin` folder containing `__init__.py` and other `.py` files is at the root of the ZIP).
2.  Open Blender.
3.  Go to `Edit > Preferences > Add-ons`.
4.  Click `Install...` and navigate to the downloaded ZIP file. Select it.
5.  Enable the plugin by checking the box next to "3D Print: AutoCut GCode for 3+2 Axis".

## How to Use (Workflow)

1.  Select your 3D model in Blender's 3D View.
2.  Open the 3D View sidebar (press 'N' if hidden) and find the **"AutoCut"** tab.

3.  **Step 1: Process Selected Planar Faces**
    *   With your model selected, go into **Edit Mode** (Tab key).
    *   Select one or more **planar faces**. These faces will define the planes for cutting and the orientation for subsequent 3+2 setups. The normal of the selected face determines the Z-axis direction for that setup.
    *   Click the **"1. Process Selected Planar Faces"** button in the AutoCut panel.

4.  **Step 2: Perform Boolean Cuts**
    *   After processing faces, this button becomes active.
    *   Click **"2. Perform Boolean Cuts"**. The plugin will create temporary cutting objects based on your selected planes and perform Boolean difference operations on your model.
    *   *Tip:* Immediately after clicking, you can adjust the `Cutter Size` and `Cutter Thickness` in the "Adjust Last Operation" panel (usually appears at the bottom-left of the 3D View, or press F9).

5.  **Step 3: Configure Settings & Export G-Code File**
    *   Adjust settings in the "Slicing Settings" and "G-Code & Extrusion Settings" sections of the AutoCut panel as needed (e.g., Layer Height, Nozzle Diameter, Feed Rates, Filament Diameter, WCS P-Number Start).
    *   Review and customize the "Custom G-Code Snippets" if necessary.
    *   Click **"3. Export G-Code File"**.
    *   A file dialog will appear. Choose a name and location for your `.gcode` file and click "Export G-Code File".

## Assumed Machine Kinematics for A/C Angles

The plugin calculates A and C rotation angles assuming a common 5-axis machine configuration where:
*   The **C-axis** rotates around the machine's primary Z-axis.
*   The **A-axis** rotates around the machine's primary X-axis.
*   The goal is to align the machine's tool Z-axis with the normal of the processed planar face for each setup.
The calculated angles are `A_deg = degrees(acos(Nz))` and `C_deg = degrees(atan2(Nx, Ny))`, where `(Nx, Ny, Nz)` is the world-space normal of the target plane.

## Limitations

*   **Basic Slicing:** The current version generates only the outer contours for each slice. It does not yet implement infill patterns or advanced wall/perimeter generation.
*   **Contour Connection:** The algorithm for connecting sliced edges into closed contours is simplified and may not produce perfect results for all complex geometries (e.g., multiple disjoint regions on a single slice, or very intricate shapes).
*   **Boolean Operations:** Boolean operations in Blender can sometimes fail or produce unexpected results on non-manifold (not "water-tight") or extremely complex meshes. Ensure your base model is clean.
*   **Extrusion Tracking in Custom Scripts:** The plugin's tracking of extruder (E) values from custom G-code snippets is basic. For reliable E value resets between orientations, ensure your custom scripts use `G92 E0` or simple, predictable `G1 E[val]` commands.

## Troubleshooting

*   If buttons are disabled, ensure you have followed the workflow steps in order (e.g., process planar faces before attempting to cut or generate G-code).
*   Check Blender's system console for error messages if something goes wrong (Window > Toggle System Console).
```
