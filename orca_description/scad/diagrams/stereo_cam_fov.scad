// Show how various cameras and lenses affect field of view

// Size of AUV
auv_size = [338, 457, 250];

// Down-facing camera tube radius
cam_r = 55;

// Height of bracket holding the camera
bracket_standoff = 5;

// Cameras are mounted on the outside of the AUV
half_baseline = auv_size.x / 2 + cam_r / 2 + bracket_standoff;
l_cam_pose = [- half_baseline, 0, 0];
r_cam_pose = [half_baseline, 0, 0];

// Draw the AUV
module auv() {
  color("#25cbf5", 0.4) translate([- auv_size.x / 2, - auv_size.y / 2, 0]) cube(auv_size);
}

// Draw a camera
module cam() {
  color("#f200ff", 0.4) cylinder(h = 150, r = cam_r / 2);
}

// Draw the field of view
module fov(horiz_fov, aspect_ratio, landscape, z) {
  x2 = tan(horiz_fov / 2) * z;
  y2 = tan(horiz_fov / 2) * z * (aspect_ratio == "4:3" ? 3 / 4 : 9 / 16);
  rotation = (landscape ? 0 : 90);
  points = [[0, 0, 0], [- x2, - y2, - z], [- x2, y2, - z], [x2, y2, - z], [x2, - y2, - z]];
  faces = [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1], [1, 2, 3, 4]];
  color("grey", 0.3) rotate([0, 0, rotation]) polyhedron(points = points, faces = faces);
}

// Draw the AUV with the cameras and the overlapping fields of view
// The default projection distance is 1.5m
module assembly(horiz_fov = 62.2, aspect_ratio = "16:9", landscape = true, z = 1500) {
  auv();
  translate(l_cam_pose) cam();
  translate(r_cam_pose) cam();
  translate(l_cam_pose) fov(horiz_fov, aspect_ratio, landscape, z);
  translate(r_cam_pose) fov(horiz_fov, aspect_ratio, landscape, z);
}

// These are all Raspicam v2's:

// Standard lens
assembly(horiz_fov = 62);

translate([0, 3000, 0]) assembly(horiz_fov = 80);

translate([0, 6000, 0]) assembly(horiz_fov = 95);

// BlueRobotics wide angle lens
translate([0, 9000, 0]) assembly(horiz_fov = 110);
