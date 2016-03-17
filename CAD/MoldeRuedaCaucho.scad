$fn = 40; // 360 es liso, para exportar

diametro_exterior = 30; // diámetro exterior de la rueda
ancho = 10; // ancho de la rueda
espesor = 5; // espesor buscado de la rueda (radio exterior - radio interior)

diametro_tornillo = 5; // diámetro de tornillo

espesor_pared_exterior = 4; // tamaño de la paredcita exterior
espesor_piso = 5; // espesor de la base del molde

radio_rueda = diametro_exterior / 2;
radio_tornillo = diametro_tornillo / 2;

// este translate mueve el borde de la pieza al punto [0, 0]
translate([radio_rueda + espesor_pared_exterior, radio_rueda + espesor_pared_exterior, 0])
difference() {
  union() {
    // pieza base contenedora
    difference() {
      // base y paredes del molde
      color("pink")    cylinder(h=(ancho + espesor_piso), r=(radio_rueda + espesor_pared_exterior));
      // hueco para el tornillo central
      color("SkyBlue") cylinder(h=(espesor_piso), r=(radio_tornillo));
      // espacio para la cubierta (esto es lo que se llenará de caucho)
      translate([0, 0, espesor_piso]) color("SkyBlue") cylinder(h=(ancho), r=(radio_rueda));
    }

    // pieza móvil del centro
    //translate([diametro_exterior, 0, -espesor_piso]) // esto saca la pieza de adentro de la otra
    translate([0, 0, espesor_piso]) // pone la pieza central justo arriba de la otra
    difference() {
      // pieza central para determinar el espesor de la rueda
      color("pink")    cylinder(h=ancho, r=(radio_rueda - espesor));
      // hueco adicional central para no requerir un tornillo muy largo
      translate([0, 0, espesor_piso]) color("SkyBlue") cylinder(h=ancho, r=(radio_rueda - espesor - espesor_pared_exterior));
      // tornillo central
      color("SkyBlue") cylinder(h=ancho, r=(radio_tornillo));
    }
  }
  // corte transversal
  //color("LightYellow") cube([150, 30, 50]);
}
