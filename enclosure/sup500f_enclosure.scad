$fn = 48;

part = "both"; // "bottom", "lid", "both"

pcb_x = 22;
pcb_y = 22;
pcb_z = 1;
pcb_overall_z = 8;

clearance = 0.3;
top_clearance = 1.0;

wall_thickness = 1.6;
floor_thickness = 1.6;
lid_thickness = 1.4;

// Lid locating lip
lid_lip_depth = 2.0;
lid_lip_clearance = 0.25;

// Rounded case corners
corner_r = 2.5;

through_hole_side = -1;
cable_side = -through_hole_side;
cable_hole_d = 5;
cable_hole_z = 4.5;

inner_x = pcb_x + clearance * 2;
inner_y = pcb_y + clearance * 2;
inner_z = pcb_overall_z + top_clearance;

outer_x = inner_x + wall_thickness * 2;
outer_y = inner_y + wall_thickness * 2;
outer_z = floor_thickness + inner_z;

lid_z = lid_thickness + lid_lip_depth;

module rounded_box(size, r) {
    x = size[0];
    y = size[1];
    z = size[2];
    linear_extrude(height = z)
        offset(r = r)
            square([x - 2*r, y - 2*r], center = true);
}

module rounded_rect_2d(size, r) {
    x = size[0];
    y = size[1];
    offset(r = r)
        square([x - 2*r, y - 2*r], center = true);
}
module cable_slot_cut() {
    translate([
        0,
        cable_side * (outer_y / 2 + 0.1),
        cable_hole_z
    ])
    rotate([90, 0, 0])
        cylinder(
            h = wall_thickness + 2,
            d = cable_hole_d,
            center = true
        );
}
module bottom_case() {
    difference() {
        // Outer body
        rounded_box([outer_x, outer_y, outer_z], corner_r);
        // Hollow cavity
        translate([0, 0, floor_thickness])
            rounded_box(
                [inner_x, inner_y, inner_z + 0.2],
                max(corner_r - wall_thickness, 0.8)
            );
        cable_slot_cut();
    }
    // Small PCB support ledge/pad
    // This keeps the board slightly off the bottom.
    translate([0, 0, floor_thickness / 2])
        cube([pcb_x - 2, pcb_y - 2, 0.6], center = true);
}

module lid() {}

bottom_case();
