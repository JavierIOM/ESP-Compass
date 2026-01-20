// ESP32 Digital Compass Case
// Designed for FireBeetle ESP32 V3.0 + Adafruit LSM303AGR + Flat Li-Po Battery
// Version 2.0 - Redesigned for flat battery under FireBeetle
//
// Print settings:
// - Layer height: 0.2mm
// - Infill: 20%
// - Supports: Not required
// - Material: PLA or PETG

/* [Case Options] */
// Generate the bottom case
generate_bottom = true;
// Generate the lid
generate_lid = true;
// Generate the platform (separate part)
generate_platform = true;

/* [Tolerances] */
// Extra clearance for board fit (adjust if too tight/loose)
board_tolerance = 0.3;
// Screw hole tolerance
screw_tolerance = 0.2;
// Gap between lid and case walls (for easy fit)
lid_gap = 0.5;

/* [FireBeetle ESP32 V3.0 Dimensions] */
fb_length = 58;          // mm
fb_width = 29;           // mm
fb_thickness = 1.6;      // mm
fb_hole_dia = 3.1;       // Mounting holes
fb_hole_x_spacing = 53;  // along length
fb_hole_y_spacing = 24;  // along width
fb_hole_x_offset = (fb_length - fb_hole_x_spacing) / 2;
fb_hole_y_offset = (fb_width - fb_hole_y_spacing) / 2;
fb_component_height = 8; // Height of tallest component on bottom of board

/* [Adafruit LSM303AGR Dimensions] */
lsm_length = 25.4;       // mm (1.0")
lsm_width = 17.7;        // mm (0.7")
lsm_thickness = 1.6;     // mm
lsm_hole_dia = 2.5;      // Mounting holes
lsm_hole_x_spacing = 20.32;
lsm_hole_y_spacing = 12.7;
lsm_hole_x_offset = (lsm_length - lsm_hole_x_spacing) / 2;
lsm_hole_y_offset = (lsm_width - lsm_hole_y_spacing) / 2;

/* [Flat Li-Po Battery Dimensions] */
battery_length = 65;     // mm
battery_width = 36;      // mm
battery_thickness = 10;  // mm
battery_tolerance = 1;   // Extra space around battery

/* [Screw Specifications] */
m3_hole = 3.2;
m3_head_dia = 6;
m3_head_height = 2;
m25_hole = 2.7;
m25_head_dia = 5;

/* [Port Cutouts] */
usb_width = 12;
usb_height = 7;

/* [Case Dimensions] */
wall_thickness = 2.5;
floor_thickness = 2;
lid_thickness = 5;      // 5mm perspex lid
lid_recess = 2;         // How deep the lid sits into the case

// Battery compartment (with tolerance)
battery_compartment_length = battery_length + battery_tolerance;
battery_compartment_width = battery_width + battery_tolerance;
battery_compartment_height = battery_thickness + 1; // +1 for clearance

// Platform above battery for FireBeetle
platform_thickness = 2;
fb_standoff_height = 6; // Standoff height for FireBeetle above platform

// Calculate positions - battery is largest, defines case size
internal_length = max(battery_compartment_length, fb_length) + 8;  // ~73mm
internal_width = max(battery_compartment_width, fb_width + lsm_width + 8) + 4; // boards side by side above battery

// Stack heights:
// Floor -> battery -> platform -> FB standoffs -> FB board -> clearance
internal_height = battery_compartment_height + platform_thickness + fb_standoff_height + fb_thickness + 10;

// External dimensions
case_length = internal_length + 2 * wall_thickness;
case_width = internal_width + 2 * wall_thickness;
case_height = internal_height + floor_thickness;

// ============================================
// MODULES
// ============================================

module rounded_box(length, width, height, radius) {
    hull() {
        translate([radius, radius, 0])
            cylinder(h=height, r=radius, $fn=32);
        translate([length-radius, radius, 0])
            cylinder(h=height, r=radius, $fn=32);
        translate([radius, width-radius, 0])
            cylinder(h=height, r=radius, $fn=32);
        translate([length-radius, width-radius, 0])
            cylinder(h=height, r=radius, $fn=32);
    }
}

module standoff(height, outer_dia, hole_dia) {
    difference() {
        cylinder(h=height, d=outer_dia, $fn=32);
        translate([0, 0, -0.1])
            cylinder(h=height+0.2, d=hole_dia, $fn=32);
    }
}

module firebeetle_standoffs() {
    positions = [
        [fb_hole_x_offset, fb_hole_y_offset],
        [fb_hole_x_offset + fb_hole_x_spacing, fb_hole_y_offset],
        [fb_hole_x_offset, fb_hole_y_offset + fb_hole_y_spacing],
        [fb_hole_x_offset + fb_hole_x_spacing, fb_hole_y_offset + fb_hole_y_spacing]
    ];
    for (pos = positions) {
        translate([pos[0], pos[1], 0])
            standoff(fb_standoff_height, m3_head_dia + 1, m3_hole + screw_tolerance);
    }
}

module lsm303_standoffs() {
    positions = [
        [lsm_hole_x_offset, lsm_hole_y_offset],
        [lsm_hole_x_offset + lsm_hole_x_spacing, lsm_hole_y_offset],
        [lsm_hole_x_offset, lsm_hole_y_offset + lsm_hole_y_spacing],
        [lsm_hole_x_offset + lsm_hole_x_spacing, lsm_hole_y_offset + lsm_hole_y_spacing]
    ];
    for (pos = positions) {
        translate([pos[0], pos[1], 0])
            standoff(fb_standoff_height, m25_head_dia + 1, m25_hole + screw_tolerance);
    }
}

module battery_compartment_walls() {
    // Corner posts to hold battery in place
    post_size = 4;
    post_height = battery_compartment_height;

    // Four corner posts
    positions = [
        [0, 0],
        [battery_compartment_length - post_size, 0],
        [0, battery_compartment_width - post_size],
        [battery_compartment_length - post_size, battery_compartment_width - post_size]
    ];

    for (pos = positions) {
        translate([pos[0], pos[1], 0])
            cube([post_size, post_size, post_height]);
    }

    // Side rails to keep battery centered
    rail_width = 2;
    rail_length = 20;

    // Left rail
    translate([battery_compartment_length/2 - rail_length/2, 0, 0])
        cube([rail_length, rail_width, post_height]);

    // Right rail
    translate([battery_compartment_length/2 - rail_length/2, battery_compartment_width - rail_width, 0])
        cube([rail_length, rail_width, post_height]);
}

module platform_with_standoffs() {
    // Platform that sits above battery, with cutout for wire routing
    // This is a SEPARATE printable part

    // FireBeetle position on platform
    fb_x = (internal_length - fb_length) / 2;
    fb_y = 4;

    // LSM303 position on platform
    lsm_x = (internal_length - lsm_length) / 2;
    lsm_y = fb_y + fb_width + 6;

    difference() {
        union() {
            // Solid platform base
            rounded_box(internal_length, internal_width, platform_thickness, 2);

            // FireBeetle standoffs (on top of platform)
            translate([fb_x, fb_y, platform_thickness])
                firebeetle_standoffs();

            // LSM303AGR standoffs (on top of platform)
            translate([lsm_x, lsm_y, platform_thickness])
                lsm303_standoffs();
        }

        // Wire routing cutout (for battery connector)
        translate([internal_length/2 - 10, internal_width/2 - 8, -0.1])
            cube([20, 16, platform_thickness + 0.2]);
    }
}

module case_bottom() {
    // Component positions (all relative to internal origin)
    battery_x = (internal_length - battery_compartment_length) / 2;
    battery_y = (internal_width - battery_compartment_width) / 2;

    platform_z = floor_thickness + battery_compartment_height;

    // FireBeetle centered above battery
    fb_x = (internal_length - fb_length) / 2;
    fb_y = 4; // Offset toward front for USB access
    fb_z = platform_z + platform_thickness;

    difference() {
        union() {
            // Main case shell
            difference() {
                rounded_box(case_length, case_width, case_height, 4);

                // Hollow interior
                translate([wall_thickness, wall_thickness, floor_thickness])
                    rounded_box(internal_length, internal_width, internal_height + 1, 2);

                // Lid recess (for 5mm perspex to sit flush)
                translate([wall_thickness/2, wall_thickness/2, case_height - lid_recess])
                    rounded_box(case_length - wall_thickness, case_width - wall_thickness, lid_recess + 1, 3);
            }

            // Battery compartment walls
            translate([wall_thickness + battery_x, wall_thickness + battery_y, floor_thickness])
                battery_compartment_walls();

            // Platform ledge supports (for the separate platform to rest on)
            platform_ledge_height = 2;
            platform_ledge_width = 3;

            // Front ledge
            translate([wall_thickness, wall_thickness, platform_z - platform_ledge_height])
                cube([internal_length, platform_ledge_width, platform_ledge_height]);

            // Back ledge
            translate([wall_thickness, wall_thickness + internal_width - platform_ledge_width, platform_z - platform_ledge_height])
                cube([internal_length, platform_ledge_width, platform_ledge_height]);

            // Left ledge
            translate([wall_thickness, wall_thickness, platform_z - platform_ledge_height])
                cube([platform_ledge_width, internal_width, platform_ledge_height]);

            // Right ledge
            translate([wall_thickness + internal_length - platform_ledge_width, wall_thickness, platform_z - platform_ledge_height])
                cube([platform_ledge_width, internal_width, platform_ledge_height]);
        }

        // USB port cutout (positioned for FireBeetle)
        usb_z = fb_z + fb_standoff_height + fb_thickness;
        translate([-1, wall_thickness + fb_y + (fb_width - usb_width)/2, usb_z])
            cube([wall_thickness + 2, usb_width, usb_height]);
    }

    // Labels engraved on floor
    translate([wall_thickness + internal_length/2, wall_thickness + internal_width/2, floor_thickness - 0.4])
        linear_extrude(0.5)
            text("3000mAh", size=5, halign="center", valign="center", $fn=32);
}

module case_lid() {
    // Simple flat lid for cutting from 5mm perspex/acrylic
    // Cut this shape from clear acrylic sheet
    perspex_thickness = 5;

    rounded_box(case_length, case_width, perspex_thickness, 4);

    // Note: No text on perspex lid - it's meant to be clear
    // Lid dimensions for cutting: ~78mm x ~62mm with 4mm corner radius
}

// ============================================
// RENDER
// ============================================

// Count how many parts are enabled
parts_enabled = (generate_bottom ? 1 : 0) + (generate_lid ? 1 : 0) + (generate_platform ? 1 : 0);

// Single part renders
if (generate_bottom && parts_enabled == 1) {
    case_bottom();
}

if (generate_lid && parts_enabled == 1) {
    case_lid();
}

if (generate_platform && parts_enabled == 1) {
    platform_with_standoffs();
}

// Multiple parts - arrange side by side
if (parts_enabled > 1) {
    x_offset = 0;

    if (generate_bottom) {
        translate([0, 0, 0])
            case_bottom();
    }

    if (generate_platform) {
        translate([generate_bottom ? case_length + 10 : 0, 0, 0])
            platform_with_standoffs();
    }

    if (generate_lid) {
        offset_x = (generate_bottom ? case_length + 10 : 0) + (generate_platform ? internal_length + 10 : 0);
        translate([offset_x, 0, 0])
            case_lid();
    }
}

// ============================================
// DIMENSIONS INFO
// ============================================
// Case external: ~78mm x ~62mm x ~37mm
// Stack order (bottom to top):
//   - Floor (2mm)
//   - Battery compartment (11mm) - holds 65x36x10mm Li-Po
//   - Platform (2mm) - SEPARATE PART, drops onto ledges
//   - Standoffs (6mm) - integrated into platform
//   - FireBeetle ESP32 + LSM303AGR side by side
//   - Clearance + Perspex lid
//
// Printable parts:
// 1. Case bottom - has battery compartment and ledges for platform
// 2. Platform - separate part with standoffs, drops onto ledges
//
// Perspex lid:
// - Cut from 5mm clear acrylic/perspex
// - Dimensions: ~78mm x ~62mm with 4mm rounded corners
// - Sits in 2mm recess around top of case
//
// Hardware needed:
// - 4x M3 screws (6-8mm) for FireBeetle
// - 4x M2.5 screws (6-8mm) for LSM303AGR
// - 1x 3000mAh flat Li-Po battery (65x36x10mm)
