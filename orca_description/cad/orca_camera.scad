// in mm

// TODO get some constants to keep sane

// Tube module -- radius is outer radius
module tube(height, radius, wall) {
  difference() {
    cylinder(h=height, r=radius);
    translate([0, 0, -1]) cylinder(h=height+2, r=radius-wall);
  }
}

// Flange module
module flange() {
  color("gray") tube(6, 57/2, 7/2+12/2);
  translate([0, 0, 6]) color("gray") difference() {
    tube(21.5-6, 50/2, 12/2);
    translate([21.5, 0, 0]) cylinder(19, 1.5); // TODO wrong depth
    translate([-21.5, 0, 0]) cylinder(19, 1.5);
    translate([0, 21.5, 0]) cylinder(19, 1.5);
    translate([0, -21.5, 0]) cylinder(19, 1.5);
  }
}

// --- main unit --

// Top flange
flange();

// Flange connector
translate([60, 0, 31/2+6]) color("blue") tube(4, 50/2, 12/2);
// TODO add screw holes

// Camera
translate([0, 0, -2]) union() {
  translate([-25/2+2.5, -10, 22.3]) cube([20, 5, 2.5]);
  translate([-25/2, -10, 22.3]) cube([25, 24.5, 1]);
  cylinder(h=22.3, r=8.3);
}

// Camera sled
//translate([2.5, 0, 59-19]) cube([25, 24.5, 2]);

// Electronics sled
//translate([2.5, 0, 59-19]) cube([15, 11, 19]);

// Pi Zero
translate([-15, -4, 35]) cube([30, 1, 65]);

// -- tube and bottom flange, attached later --

// Bottom flange
translate([0, 0, 132]) rotate([0, 180, 0]) flange();

// Tube
color("white", 0.1) translate([0, 0, 6]) tube(120, 57.2/2, 3.2);