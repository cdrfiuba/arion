$fn = 40; // 360 is smooth

espesor = 10; // espesor buscado de la rueda (radio exterior - radio interior)
diametro_exterior = 60; // diámetro exterior de la rueda
radio_rueda = diametro_exterior / 2;
ancho = 10; // ancho de la rueda
diametro_tornillo = 5; // diámetro de tornillo
radio_tornillo = diametro_tornillo / 2; // diámetro de tornillo
espesor_pared_exterior = 4; // tamaño de la paredcita exterior
espesor_piso = 5; // espesor de la base del molde

translate([radio_rueda + espesor_pared_exterior, radio_rueda + espesor_pared_exterior, 0])
difference() {
  union() {
    // pieza base contenedora
    difference() {
      // base y paredes del molde
      union () {
        color("pink")    cylinder(h=(ancho + espesor_piso), r=(radio_rueda + espesor_pared_exterior));

        // paredcita exterior
        difference () {
          color("pink")    cylinder(h=(ancho + espesor_piso + 3), r=(radio_rueda + espesor_pared_exterior));
          color("SkyBlue") cylinder(h=(ancho + espesor_piso + 4), r=(radio_rueda + espesor_pared_exterior / 2));
        }
      }
      // espacio para la cubierta
      translate([0, 0, espesor_piso]) color("SkyBlue") cylinder(h=(ancho + espesor_piso), r=(radio_rueda));
      // tornillo central
      color("SkyBlue") cylinder(h=(ancho + espesor_piso), r=(radio_tornillo));
    }

    // pieza móvil del centro
    //translate([(diametro_exterior * 2), 0, -espesor_piso]) // esto saca la pieza de adentro de la otra
    translate([0, 0, espesor_piso]) // pone la pieza central justo arriba de la otra
    difference() {
      // pieza central para determinar el ancho de la rueda
      union () {
        color("pink")    cylinder(h=ancho, r=(radio_rueda - espesor));
        // paredcita interior
        difference () {
          color("pink")    cylinder(h=(ancho + espesor_piso), r=(radio_rueda - espesor - espesor_pared_exterior / 2));
          color("SkyBlue") cylinder(h=(ancho + espesor_piso), r=(radio_rueda - espesor - espesor_pared_exterior));
        }
      }
      // hueco adicional para no requerir un tornillo muy largo
      translate([0, 0, espesor_piso]) color("SkyBlue") cylinder(h=ancho, r=(radio_rueda - espesor - espesor_pared_exterior));
      // tornillo central
      color("SkyBlue") cylinder(h=ancho, r=(radio_tornillo));
    }
  }
  // corte transversal
  //color("LightYellow") cube([150, 30, 50]);
}
