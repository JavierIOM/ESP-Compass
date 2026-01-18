// ESP32 Digital Compass Case
// Designed for FireBeetle ESP32 V3.0 + Adafruit LSM303AGR + 18650 Battery
// Version 1.0
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

/* [Adafruit LSM303AGR Dimensions] */
lsm_length = 25.4;       // mm (1.0")
lsm_width = 17.7;        // mm (0.7")
lsm_thickness = 1.6;     // mm
lsm_hole_dia = 2.5;      // Mounting holes
lsm_hole_x_spacing = 20.32;
lsm_hole_y_spacing = 12.7;
lsm_hole_x_offset = (lsm_length - lsm_hole_x_spacing) / 2;
lsm_hole_y_offset = (lsm_width - lsm_hole_y_spacing) / 2;

/* [18650 Battery Dimensions] */
// MUST BE DEFINED BEFORE Case Dimensions!
battery_diameter = 18.5;   // mm (with tolerance)
battery_length = 65.5;     // mm (with tolerance)
battery_holder_wall = 2;   // Wall thickness for battery cradle

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
standoff_height = 6;
lid_height = 8;
lid_lip = 1.5;

// Cradle dimensions (calculated from battery)
cradle_width = battery_diameter + 2 * battery_holder_wall;  // ~22.5mm
cradle_height = battery_diameter / 2 + battery_holder_wall;  // ~11.25mm (lowered)

// Internal dimensions - sized for all components
internal_length = battery_length + 10;  // ~75.5mm (battery is longest)
internal_width = fb_width + 8 + lsm_width + 8 + cradle_width + 5;  // boards + gaps + battery + margin
internal_height = max(standoff_height + fb_thickness + 12, cradle_height + 5);

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
            standoff(standoff_height, m3_head_dia + 1, m3_hole + screw_tolerance);
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
            standoff(standoff_height, m25_head_dia + 1, m25_hole + screw_tolerance);
    }
}

module battery_cradle() {
    // Simple U-shaped cradle - solid block with half-cylinder carved out
    difference() {
        // Solid base block
        cube([battery_length, cradle_width, cradle_height]);

        // Carve U-channel for battery (half-cylinder from top)
        translate([-1, cradle_width/2, cradle_height])
            rotate([0, 90, 0])
                cylinder(h=battery_length + 2, d=battery_diameter, $fn=48);
    }

    // End lips to retain battery
    lip_height = 3;
    lip_width = battery_holder_wall;

    // Left lip
    translate([0, battery_holder_wall, cradle_height])
        cube([lip_width, battery_diameter, lip_height]);

    // Right lip
    translate([battery_length - lip_width, battery_holder_wall, cradle_height])
        cube([lip_width, battery_diameter, lip_height]);
}

module case_bottom() {
    // Component positions (all relative to case origin)
    fb_x = wall_thickness + (internal_length - fb_length) / 2;
    fb_y = wall_thickness + 5;

    lsm_x = wall_thickness + (internal_length - lsm_length) / 2;
    lsm_y = fb_y + fb_width + 8;

    battery_x = wall_thickness + (internal_length - battery_length) / 2;
    battery_y = lsm_y + lsm_width + 8;

    difference() {
        union() {
            // Main case shell
            difference() {
                rounded_box(case_length, case_width, case_height, 4);

                // Hollow interior
                translate([wall_thickness, wall_thickness, floor_thickness])
                    rounded_box(internal_length, internal_width, internal_height + 1, 2);

                // Lid lip recess
                translate([wall_thickness - lid_lip, wall_thickness - lid_lip, case_height - lid_height])
                    rounded_box(internal_length + 2*lid_lip, internal_width + 2*lid_lip, lid_height + 1, 2);
            }

            // FireBeetle standoffs
            translate([fb_x, fb_y, floor_thickness])
                firebeetle_standoffs();

            // LSM303AGR standoffs
            translate([lsm_x, lsm_y, floor_thickness])
                lsm303_standoffs();

            // Battery cradle
            translate([battery_x, battery_y, floor_thickness])
                battery_cradle();
        }

        // USB port cutout
        translate([-1, fb_y + (fb_width - usb_width)/2, floor_thickness + standoff_height + fb_thickness])
            cube([wall_thickness + 2, usb_width, usb_height]);
    }

    // Labels engraved on floor
    translate([fb_x + fb_length/2, fb_y + fb_width/2, floor_thickness - 0.4])
        linear_extrude(0.5)
            text("ESP32", size=5, halign="center", valign="center", $fn=32);

    translate([lsm_x + lsm_length/2, lsm_y + lsm_width/2, floor_thickness - 0.4])
        linear_extrude(0.5)
            text("MAG", size=4, halign="center", valign="center", $fn=32);

    translate([battery_x + battery_length/2, battery_y + cradle_width/2, floor_thickness - 0.4])
        linear_extrude(0.5)
            text("18650", size=4, halign="center", valign="center", $fn=32);
}

module case_lid() {
    // Lid top plate
    rounded_box(case_length, case_width, 2, 4);

    // Inner lip (fits into case recess)
    translate([wall_thickness - lid_lip + lid_gap, wall_thickness - lid_lip + lid_gap, 2])
        rounded_box(internal_length + 2*lid_lip - 2*lid_gap,
                   internal_width + 2*lid_lip - 2*lid_gap,
                   lid_height - 1, 1.5);

    // Text on lid
    translate([case_length/2, case_width/2, 2 - 0.3])
        linear_extrude(0.4)
            text("ESP32 COMPASS", size=6, halign="center", valign="center", $fn=32);

    translate([case_length/2, case_width/2 - 10, 2 - 0.3])
        linear_extrude(0.4)
            text("v0.0.4", size=4, halign="center", valign="center", $fn=32);
}

// ============================================
// RENDER
// ============================================

if (generate_bottom && !generate_lid) {
    case_bottom();
}

if (generate_lid && !generate_bottom) {
    case_lid();
}

if (generate_bottom && generate_lid) {
    case_bottom();
    translate([case_length + 10, 0, 0])
        case_lid();
}

// ============================================
// DIMENSIONS INFO
// ============================================
// Case external: ~81mm x ~99mm x ~27mm
// Hardware needed:
// - 4x M3 screws (6-8mm) for FireBeetle
// - 4x M2.5 screws (6-8mm) for LSM303AGR
// - 1x 18650 battery
