$fn = 40; // 360 is smooth

module tire(radio, thickness, width) {
  translate([radio / 2, radio / 2, thickness * 2])
  rotate_extrude()
    translate([radio / 2 + thickness, 0, 0])
      translate([-thickness * 2, -thickness * 2, 0]) 
        square([thickness, width]);  
}
//color("lightgray") tire(60, 5, 20);

//// prototipo con module tire
//difference() {
//  translate([35 - 5, 35 - 5, 0]) color("pink")    cylinder(h=25, r=35);
//  translate([0, 0, 5])           color("SkyBlue") tire(60, 10, 20);
//  translate([35 - 5, 35 - 5, 0]) color("SkyBlue") cylinder(h=25, r=2);
//}

// pieza base contenedora
difference() {
  // base y paredes del molde
  translate([30, 30, 0]) color("pink")    cylinder(h=25, r=30);
  // espacio para la cubierta
  translate([30, 30, 5]) color("SkyBlue") cylinder(h=25, r=25);
  // tornillo central
  translate([30, 30, 0]) color("SkyBlue") cylinder(h=25, r=2);
}

// pieza móvil del centro
translate([70, 0, -5]) // esto saca la pieza de adentro de la otra
difference() {
  // pieza central para determinar el ancho de la rueda
  translate([30, 30, 5]) color("pink")    cylinder(h=20, r=15);
  // tornillo central
  translate([30, 30, 5]) color("SkyBlue") cylinder(h=20, r=2);
}
