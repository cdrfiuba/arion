$fn = 40; // 360 is smooth

difference() {
  union() {
    // pieza base contenedora
    difference() {
      // base y paredes del molde
      union () {
        translate([30, 30, 0]) color("pink")    cylinder(h=25, r=30);
        // paredcita exterior
        difference () {
          translate([30, 30, 0]) color("pink")    cylinder(h=28, r=30);
          translate([30, 30, 0]) color("SkyBlue") cylinder(h=28, r=27.5);
        }
      }
      // espacio para la cubierta
      translate([30, 30, 5]) color("SkyBlue") cylinder(h=25, r=25);
      // tornillo central
      translate([30, 30, 0]) color("SkyBlue") cylinder(h=25, r=2.5);
    }

    // pieza móvil del centro
    translate([60, 0, -5]) // esto saca la pieza de adentro de la otra
    difference() {
      // pieza central para determinar el ancho de la rueda
      union () {
        translate([30, 30, 5])  color("pink")    cylinder(h=20, r=15);
        // paredcita interior
        difference () {
          translate([30, 30, 5]) color("pink")    cylinder(h=23, r=12.5);
          translate([30, 30, 5]) color("SkyBlue") cylinder(h=23, r=10);
        }
      }
      // hueco adicional para no requerir un tornillo muy largo
      translate([30, 30, 10]) color("SkyBlue") cylinder(h=20, r=10);
      // tornillo central
      translate([30, 30, 5])  color("SkyBlue") cylinder(h=20, r=2.5);
    }
  }
  // corte transversal
  //color("LightYellow") cube([150, 30, 50]);
}