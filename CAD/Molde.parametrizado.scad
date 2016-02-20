$fn = 40; // 360 is smooth

espesor = 5; //espesor buscado de la rueda (radio ext- radio int)
Dext = 30; //diametro exterior de la rueda
Ancho = 10; //ancho de la rueda
Dtornillo=5; //diametro de tornillo

difference() {
  union() {
    // pieza base contenedora
    difference() {
      // base y paredes del molde
      union () {
        //translate([30, 30, 0]) color("pink")    cylinder(h=25, r=30);

		translate([30, 30, 0]) color("pink")    cylinder(h=(Ancho+5), r=(Dext/2+4));        

		// paredcita exterior
        difference () {
          translate([30, 30, 0]) color("pink")    cylinder(h=(Ancho+8), r=(Dext/2+4));
          translate([30, 30, 0]) color("SkyBlue") cylinder(h=(Ancho+9), r=(Dext/2+2));
        }
      }
      // espacio para la cubierta
      translate([30, 30, 5]) color("SkyBlue") cylinder(h=(Ancho+5), r=(Dext/2+0));
      // tornillo central
      translate([30, 30, 0]) color("SkyBlue") cylinder(h=(Ancho+5), r=(Dtornillo/2));
    }

    // pieza móvil del centro
    translate([(Dext*2), 0, -5]) // esto saca la pieza de adentro de la otra
    difference() {
      // pieza central para determinar el ancho de la rueda
      union () {
        translate([30, 30, 5])  color("pink")    cylinder(h=Ancho, r=(Dext/2-espesor));
        // paredcita interior
        difference () {
          translate([30, 30, 5]) color("pink")    cylinder(h=(Ancho+3), r=(Dext/2-espesor-2));
          translate([30, 30, 5]) color("SkyBlue") cylinder(h=(Ancho+3), r=(Dext/2-espesor-4));
        }
      }
      // hueco adicional para no requerir un tornillo muy largo
      translate([30, 30, 10]) color("SkyBlue") cylinder(h=Ancho, r=(Dext/2-espesor-4));
      // tornillo central
      translate([30, 30, 5])  color("SkyBlue") cylinder(h=Ancho, r=(Dtornillo/2));
    }
  }
  // corte transversal
  //color("LightYellow") cube([150, 30, 50]);
}