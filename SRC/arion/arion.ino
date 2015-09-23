// nada marron nada blanco rojo nada
//              micro

// nada nada blanco rojo marron
//            usb

char debug_string_buffer[50];
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer); \
  delay(1);

// Configuracion de debug
const bool DEBUG = false;
const bool DEBUG_SETPOINT = false;
const bool DEBUG_PIDM = false;

// Constantes no tocar ////////////////////////
const int atras = HIGH;
const int adelante = LOW;
const int haciaIzquierda = 0;
const int haciaDerecha = 1;

// definición de pines del micro ////////////////////////
const int pwmMotorD = 11;
const int pwmMotorI = 10;
const int sentidoMotorD = 3;
const int sentidoMotorI = 5;
const int ledArduino = 13;
const int led1 = 9;
const int led2 = 8;
const int led3 = 12;
const int boton1 = 7;
const int boton2 = 6;
const int boton3 = 4;
const int sensor0 = A0;
const int sensor1 = A1;
const int sensor2 = A2;
const int sensor3 = A3;
const int sensor4 = A5;
const int batteryControl = A4;

// para batería
// 8.23 V => 847
// 8.00 V => 822
// 7.50 V  => 771
const int MINIMO_VALOR_BATERIA = 771;

// Variables de sensores infrarrojos ////////////////////////
const int cantidadDeSensores = 5;
int sensores[cantidadDeSensores];
// guarda los valores mínimos y máximos de calibración
int minimosSensores[cantidadDeSensores];
int maximosSensores[cantidadDeSensores];
// indices de array sensores
const int izq    = 0;
const int cenIzq = 1;
const int cen    = 2;
const int cenDer = 3;
const int der    = 4;



// Variables usadas en el algoritmo de control ////////////////////////
//// Variables para el modo "me fui" ////
bool estadoActualAdentro; // determina si se usa modo PID o modo "me fui"
const int derecha = 1; // No tocar: bordes para modo "me fui"
const int izquierda = 0; // No tocar: bordes para modo "me fui"
const int tolerancia = 0; // Margen de ruido al medir negro.
const int toleranciaBorde = 200; // Mínimo para decidir cuál fue el último borde
int ultimoBorde;

// para calcular tiempo entre ciclos de PID.
// no debe ser 0, pues se usa para dividir
long int ultimoTiempoUs = 0; // guarda el valor de micros()
int tiempoUs = 0; // guarda el tiempo del ciclo
const int tiempoCicloReferencia = 1290; // TODO habria que recalcularlo

//// Variables PID principal ////
// velocidadMinima + rangoVelocidad <= 255 (o explota)
//OBSOLETE const int velocidadMinima = 10;
//OBSOLETE const int velocidadFreno = 25; //50
const int rangoVelocidad = 70; //120
int reduccionVelocidad;
int errP, errD, errPAnterior;
long errI; // errores integrales deben ser long
int velocidadMotorFrenado, velocidadMotorNoFrenado;
int velocidadPenalizacion; // TODO: deberia ser un coeficiente float y no algo que se resta
int direccionMovimientoLateral;
int sensoresLinea = 0; // valor analogico de los sensores combinados
const int centroDeLinea = 3000; // depende de una formula de los sensores combinados (que esta abajo)


//// Variables PID de motores ////
int motorISetPoint, motorDSetPoint; 
int motorIzqPIDout, motorDerPIDout; // salida real a motores
int motorIzqPIDoutAnterior, motorDerPIDoutAnterior; // salida real a motores en el ciclo anterior
int motorIzqErrP, motorIzqErrPAnterior, motorIzqErrD; // variables de errores motor Izquierdo
int motorDerErrP, motorDerErrPAnterior, motorDerErrD; // variables de errores motor Derecho
long motorIzqErrI, motorDerErrI; // errores integrales deben ser long
int pwmAvgMotorI, pwmAvgMotorD; // estimadores de RPM expresado en PWM
int expAvgWeight = 950;  // 1000 = creo solo en el pasado; 0 = creo solo en lo nuevo
const int motorIzqRateLimit = 3; // Maximo incremento ciclo a ciclo de control. TODO deberia considerar tiempo y no ciclo a ciclo
const int motorDerRateLimit = 3; // Maximo incremento ciclo a ciclo de control. TODO deberia considerar tiempo y no ciclo a ciclo

// Tests
int cant_ciclos_test = 400;
int num_ciclo_test = 0;
int setpoint_test = 70;

// Parametrizacion //////////////////////////////////////
// PID General
const int coeficienteErrorPmult = 1; // 1
const int coeficienteErrorPdiv = 5;  // 7
const int coeficienteErrorIdiv = 32000; // 2500
const int coeficienteErrorDmult = 8; // 8
const int coeficienteErrorDdiv = 1; // 1

// Parametros PID motor Izquierdo
const int coefMotorIErrPmult = 20;
const int coefMotorIErrPdiv  = 1;
const int coefMotorIErrDmult = 20;
const int coefMotorIErrDdiv  = 1;
const int coefMotorIErrIdiv  = 32000; // Valor alto para desactivar efecto

// Parametros PID motor Derecho
const int coefMotorDErrPmult = 20;
const int coefMotorDErrPdiv = 1;
const int coefMotorDErrDmult = 20;
const int coefMotorDErrDdiv = 1;
const int coefMotorDErrIdiv = 32000; // Valor alto para desactivar efecto



void setup() {
  // como los motores se manejan con AnalogWrite,
  // no hace falta ponerlos como salida
  // pinMode(pwmMotorD, OUTPUT);
  // pinMode(pwmMotorI, OUTPUT);

  pinMode(sentidoMotorD, OUTPUT);
  pinMode(sentidoMotorI, OUTPUT);

  // como los sensores y el batteryControl se leen con AnalogRead,
  // no hace falta ponerlos como entrada
  //pinMode(sensor0, INPUT);
  //pinMode(sensor1, INPUT);
  //pinMode(sensor2, INPUT);
  //pinMode(sensor3, INPUT);
  //pinMode(sensor4, INPUT);
  //pinMode(batteryControl, INPUT);

  pinMode(ledArduino, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  pinMode(boton1, INPUT);
  pinMode(boton2, INPUT);
  pinMode(boton3, INPUT);

  if (DEBUG) {
    Serial.begin(9600);
  }

  for (int i = 0; i < cantidadDeSensores; i++) {
    sensores[i] = 0;
    minimosSensores[i] = 0;
    maximosSensores[i] = 1023;
  }

  apagarMotores();

  // Inicializacion variables PID general
  errP = 0; // No es necesario
  errI = 0;
  errD = 0;
  errPAnterior = 0;
  velocidadPenalizacion = 0; // Va de 0 a rangoVelocidad
  
  // Inicializacion variables "modo afuera"
  ultimoBorde = izquierda;
  estadoActualAdentro = true;
  
  // Inicializacion variables PIDs de motores
  pwmAvgMotorI = 0;
  pwmAvgMotorD = 0;
  motorIzqErrI = 0;
  motorDerErrI = 0;
  motorIzqPIDoutAnterior = 0;
  motorDerPIDoutAnterior = 0;

  // Otras inicializaciones
  tiempoUs = tiempoCicloReferencia; // NM: porque 1.0? y encima es un int
  ultimoTiempoUs = 0;

  // Inicializacion test
  setpoint_test = rangoVelocidad;


}

bool apretado(int boton) {
  return (digitalRead(boton) == LOW);
}

void esperarReboteBoton() {
  delay(5);
}

inline void obtenerSensores() {
  // carga en el array de sensores las lecturas AD de cada sensor
  // este proceso lleva 500 us
  sensores[izq]    = 1024 - analogRead(sensor0);
  sensores[cenIzq] = 1024 - analogRead(sensor1);
  sensores[cen]    = 1024 - analogRead(sensor2);
  sensores[cenDer] = 1024 - analogRead(sensor3);
  sensores[der]    = 1024 - analogRead(sensor4);
}

inline void obtenerSensoresCalibrados() {
  int denominador = 0;
  signed int valor = 0;
  obtenerSensores();
  for (int i = 0; i < cantidadDeSensores; i++) {
    denominador = maximosSensores[i] - minimosSensores[i];
    valor = ((signed long)sensores[i] - minimosSensores[i]) * 1023 / denominador;
    if (valor > 1023) {
      valor = 1023;
    } else if (valor < 0) {
      valor = 0;
    }
    sensores[i] = valor;
  }
}

void calibrarSensores() {
  // reseteo la calibración
  for (int i = 0; i < cantidadDeSensores; i++) {
    minimosSensores[i] = 1023;
    maximosSensores[i] = 0;
  }

  // leo los sensores, y guardo los mínimos y los máximos
  obtenerSensores();
  for (int i = 0; i < cantidadDeSensores; i++) {
    if (sensores[i] < minimosSensores[i]) {
      minimosSensores[i] = sensores[i];
    }
    if (sensores[i] > minimosSensores[i]) {
      maximosSensores[i] = sensores[i];
    }
  }
}

void mostrarSensorLEDs(int sensor) {
  if ((sensor >= cantidadDeSensores) || (sensor < 0)) {
    return;
  }
  digitalWrite(led1, ((sensores[sensor] /  768) ? HIGH : LOW));
  digitalWrite(led2, ((sensores[sensor] /  512) ? HIGH : LOW));
  digitalWrite(led3, ((sensores[sensor] /  256) ? HIGH : LOW));
}

void mostrarSensores() {
  if (DEBUG) { // Sin puerto serie no tiene sentido
    debug("%.4d ", analogRead(batteryControl));
    debug("%.4d ", sensores[izq]);
    debug("%.4d ", sensores[cenIzq]);
    debug("%.4d ", sensores[cen]);
    debug("%.4d ", sensores[cenDer]);
    debug("%.4d\n", sensores[der]);
  }
}

void apagarMotores() {
  analogWrite(pwmMotorD, 0);
  analogWrite(pwmMotorI, 0);
  digitalWrite(sentidoMotorI, adelante);
  digitalWrite(sentidoMotorD, adelante);

}

inline void chequearBateria() {
  if (analogRead(batteryControl) < MINIMO_VALOR_BATERIA) {
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
  } else {
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
  }
}
inline void chequearBateriaBloqueante() {
  if (analogRead(batteryControl) < MINIMO_VALOR_BATERIA) {
    while (!apretado(boton1)) {
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      digitalWrite(led3, HIGH);
      delay(200);
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      delay(200);
    }
    esperarReboteBoton();
  }
}

void loop() {

  /*
  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
  delay(1000);
  digitalWrite(led1, LOW);
  delay(1000);
  digitalWrite(led2, LOW);
  delay(1000);
  digitalWrite(led3, LOW);

  for (int i = 0; i < (velocidadMinima + rangoVelocidad) / 10; i++) {
    analogWrite(pwmMotorD, i * 10);
    analogWrite(pwmMotorI, i * 10);
    delay(100);
  }

  analogWrite(pwmMotorI, velocidadMinima + rangoVelocidad);
  analogWrite(pwmMotorD, velocidadMinima + rangoVelocidad);
  delay(500);

  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
  digitalWrite(sentidoMotorI, HIGH);
  digitalWrite(sentidoMotorD, HIGH);
  delay(10);
  digitalWrite(sentidoMotorI, LOW);
  digitalWrite(sentidoMotorD, LOW);
  analogWrite(pwmMotorI, velocidadMinima + rangoVelocidad);
  analogWrite(pwmMotorD, velocidadMinima);

  while(1);

  */

  /*
    while (1) {
      obtenerSensores();
      mostrarSensores();
      delay(50);
      //debug("%.4d ", analogRead(A4));
      //debug("%.4d ", analogRead(A5));
      //debug("%.4d\n ", analogRead(A6));
    }
  */
  /*
    while(1) {

      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      digitalWrite(led3, HIGH);

      while (!apretado(boton2));
      esperarReboteBoton();

      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      while (apretado(boton2));

    }

  */
  // inicialización de todo
  setup();

  // hasta que se presione el botón, espera,
  // y muestra en los leds el valor del sensor central
  while (!apretado(boton1)) {
    obtenerSensoresCalibrados();
    mostrarSensorLEDs(cen);
    mostrarSensores();
    chequearBateriaBloqueante();

    while (apretado(boton3)) {
      digitalWrite(led1, HIGH);
      calibrarSensores();
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      delay(50);
    }
  }
  esperarReboteBoton();
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);

  // hasta que se suelte el botón, espera
  while (apretado(boton1));
  esperarReboteBoton();

  // Arranque suave de los motores (a ciegas en linea recta)
  /*
  for (int i = 0; i < rangoVelocidad / 10; i++) {
    analogWrite(pwmMotorD, i * 10);
    analogWrite(pwmMotorI, i * 10);
    delay(50);
  }
  // TODO calcular valores iniciales apropiados luego del arranque ciego
  // TODO eliminar el arranque ciego, el mismo rate limiter podria encargarse de eso
  */

  // ejecuta el ciclo principal hasta que se presione el botón
  while (!apretado(boton1)) {
    obtenerSensoresCalibrados();

    // Se determina cual fue el ultimo lado por el que se vio la linea blanca
    if (sensores[izq] > toleranciaBorde) {
      ultimoBorde = izquierda;
    } else if (sensores[der] > toleranciaBorde) {
      ultimoBorde = derecha;
    }

    // si me fui, entro en modo "corrección máxima"
    if ((sensores[izq]    < tolerancia) &&
        (sensores[cenIzq] < tolerancia) &&
        (sensores[cen]    < tolerancia) &&
        (sensores[cenDer] < tolerancia) &&
        (sensores[der]    < tolerancia)) {
      estadoActualAdentro = false;
    } else {
      estadoActualAdentro = true;
    }

    if (estadoActualAdentro) {
      // modo PID
      // OLD: linea = (0 * s0 + 1000 * s1 + 2000 * s2 + 3000 * s3 + 4000 * s4) / (s0 + s1 + s2 + s3 + s4)
      // 0 a 4000, donde 2000 es el centroDeLinea
      // Se calcula el valor analogico combinado de los sensores
      // 0 a 6000, donde 3000 es el centroDeLinea
      // NEW: linea = (0 * s0 + 2000 * s1 + 3000 * s2 + 4000 * s3 + 6000 * s4) / (s0 + s1 + s2 + s3 + s4)
      
      sensoresLinea = (
                        (long)sensores[izq]    * 0 +
                        (long)sensores[cenIzq] * 2000 +
                        (long)sensores[cen]    * 3000 +
                        (long)sensores[cenDer] * 4000 +
                        (long)sensores[der]    * 6000
                      ) / (
                        (long)sensores[izq]    +
                        (long)sensores[cenIzq] +
                        (long)sensores[cen]    +
                        (long)sensores[cenDer] +
                        (long)sensores[der]
                      );

      // Cuentas PID
      errP = sensoresLinea - centroDeLinea;
      errI += (errP * tiempoCicloReferencia) / tiempoUs;
      errD = ((errP - errPAnterior) * tiempoUs) / tiempoCicloReferencia;
      // TODO implementar caja negra para reportar overflow
      errPAnterior = errP;
      reduccionVelocidad = (errP * coeficienteErrorPmult) / coeficienteErrorPdiv  + (errD * coeficienteErrorDmult) / coeficienteErrorDdiv + errI / coeficienteErrorIdiv;

      // COSAS VIEJAS
      // errP va entre -2000 y 2000, con p=1/12 reduccionVelocidad va entre -166 y +166
      // errD va entre -4000 y 4000, con d=1/30 reduccionVelocidad va entre -133 y +133
      // // err_i toma valores entre -32k y 32k, por lo que su aporte a diff_potencia esta acotado entre -32 y +32 (-32 y +32 para 6 sensores)
      // // err_d toma valores entre -5k y 5k, por lo que su aporte a diff_potencia esta acotado entre -inf y +inf (para los niveles de representacion que manejamos).
      // // Para un caso normal, en que err_p varie 30 entre una medicion y la siguiente, estará acotado entre -45 y +45

      // constrains de actuacion
      reduccionVelocidad  = constrain(reduccionVelocidad, - (rangoVelocidad - velocidadPenalizacion), rangoVelocidad - velocidadPenalizacion); 
      /*
      if (reduccionVelocidad < - (rangoVelocidad - velocidadPenalizacion) ) {
        reduccionVelocidad = -rangoVelocidad;
      } else if (reduccionVelocidad > rangoVelocidad - velocidadPenalizacion ) {
        reduccionVelocidad = rangoVelocidad;
      }
      */

      // Se determina el sentido de giro
      if (reduccionVelocidad < 0) {
        direccionMovimientoLateral = haciaIzquierda;
      } else {
        direccionMovimientoLateral = haciaDerecha;
      }

      // Calculo del setting point de los PIDs de motores
      reduccionVelocidad = abs(reduccionVelocidad);
      velocidadMotorFrenado = rangoVelocidad - velocidadPenalizacion - reduccionVelocidad;
      velocidadMotorNoFrenado = rangoVelocidad - velocidadPenalizacion;

      if (direccionMovimientoLateral == haciaIzquierda) {
        motorISetPoint = velocidadMotorFrenado;
        motorDSetPoint = velocidadMotorNoFrenado;
      } else if (direccionMovimientoLateral == haciaDerecha) {
        motorISetPoint = velocidadMotorNoFrenado;
        motorDSetPoint = velocidadMotorFrenado;
      }


      // HARDCODE PID MOTORES TEST
      /*
      if (num_ciclo_test++ > cant_ciclos_test) {
        num_ciclo_test = 0;
        if ( setpoint_test  == rangoVelocidad)
          setpoint_test = 0;
        else
          setpoint_test = rangoVelocidad;
      }
      motorISetPoint = setpoint_test;
      motorDSetPoint = setpoint_test;
      */



      if (DEBUG_SETPOINT) {
        debug("SP I:% .4i ", motorISetPoint);
        debug("D:% .4i\n", motorDSetPoint);
      }


      // PID motor Izquierdo
      motorIzqErrP = motorISetPoint - ((pwmAvgMotorI + 64) >> 7);
      motorIzqErrI += (motorIzqErrP * tiempoCicloReferencia) / tiempoUs;
      motorIzqErrD = (motorIzqErrP - motorIzqErrPAnterior) * tiempoUs / tiempoCicloReferencia;

      motorIzqErrPAnterior = motorIzqErrP;

      motorIzqPIDout = (motorIzqErrP * coefMotorIErrPmult) / coefMotorIErrPdiv  + (motorIzqErrD * coefMotorIErrDmult) / coefMotorIErrDdiv + motorIzqErrI / coefMotorIErrIdiv;
//      motorIzqPIDout = constrain(motorIzqPIDout, - rangoVelocidad, rangoVelocidad - velocidadPenalizacion); // No penalizo para frenar

      if (DEBUG_PIDM) {
        debug("PMI % .4i ", motorIzqErrP);
        debug("% .10li ", motorIzqErrI);
        debug("% .4i ", motorIzqErrD);
        debug("% .4i\n", motorIzqPIDout);
      }
     

      // PID motor Derecho
      motorDerErrP = motorDSetPoint - ((pwmAvgMotorD + 64) >> 7);
      motorDerErrI += (motorDerErrP * tiempoCicloReferencia) / tiempoUs;
      motorDerErrD = (motorDerErrP - motorDerErrPAnterior ) * tiempoUs / tiempoCicloReferencia;

      motorDerErrPAnterior = motorDerErrP;

      motorDerPIDout = (motorDerErrP * coefMotorDErrPmult) / coefMotorDErrPdiv  + (motorDerErrD * coefMotorDErrDmult) / coefMotorDErrDdiv + motorDerErrI / coefMotorDErrIdiv;
//      motorDerPIDout = constrain(motorDerPIDout, - rangoVelocidad, rangoVelocidad - velocidadPenalizacion); // No penalizo para frenar

      if (DEBUG_PIDM) {
        debug("PMD % .4i ", motorDerErrP);
        debug("% .10li ", motorDerErrI);
        debug("% .4i ", motorDerErrD);
        debug("% .4i\n", motorDerPIDout);
      }
      
      // Rate Limit
      motorIzqPIDout = constrain(motorIzqPIDout, motorIzqPIDoutAnterior - motorIzqRateLimit, motorIzqPIDoutAnterior + motorIzqRateLimit);
      motorDerPIDout = constrain(motorDerPIDout, motorDerPIDoutAnterior - motorDerRateLimit, motorDerPIDoutAnterior + motorDerRateLimit);

      // Restriccion de rango dinamico
      motorIzqPIDout = constrain(motorIzqPIDout, -255, 255);
      motorDerPIDout = constrain(motorDerPIDout, -255, 255);

      // Almaceno valor anterior 
      motorIzqPIDoutAnterior = motorIzqPIDout;
      motorDerPIDoutAnterior = motorDerPIDout;

      if (DEBUG_PIDM) {
        debug("M I:% .4i ", motorIzqPIDout);
        debug("D:% .4i\n", motorDerPIDout);
      }


      // Aplico acciones Motor Izquierdo
      if (motorIzqPIDout < 0 ) { // Frenar
        digitalWrite(sentidoMotorI, atras);
        analogWrite(pwmMotorI, 255 - abs(motorIzqPIDout));
      } else {
        digitalWrite(sentidoMotorI, adelante);
        analogWrite(pwmMotorI, motorIzqPIDout);
      }
      // Aplico acciones Motor Derecho
      if (motorDerPIDout < 0 ) { // Frenar
        digitalWrite(sentidoMotorD, atras);
        analogWrite(pwmMotorD, 255 - abs(motorDerPIDout));
      } else {
        digitalWrite(sentidoMotorD, adelante);
        analogWrite(pwmMotorD, motorDerPIDout);
      }

      // Estimador de "RPM" en unidades de PWM 
      // (i.e. suponemos que hay una relacion entre las RPM estacionarias para un dado PWM
      // que considera la carga que genera la masa del robot <> NoLoad) 
      pwmAvgMotorI = ((long)expAvgWeight * (long)pwmAvgMotorI) / 1000 + ((1000 - expAvgWeight) * (long)(motorIzqPIDout << 7)) / 1000;
      pwmAvgMotorD = ((long)expAvgWeight * (long)pwmAvgMotorD) / 1000 + ((1000 - expAvgWeight) * (long)(motorDerPIDout << 7)) / 1000;

      if (DEBUG_PIDM) {
        debug("EST I:% .4i ", (pwmAvgMotorI + 64) >> 7);
        debug("D:% .4i\n", (pwmAvgMotorD + 64) >> 7 );
      }


      // tiempoUs = (double)micros() / 1000.0 - tiempoUs;
      if (DEBUG) {
        tiempoUs = micros() - ultimoTiempoUs;
        debug("%.4i ", tiempoUs);
        debug("% .4i ", sensoresLinea);
        debug("% .4i ", errP);
        debug("% .3i\n", reduccionVelocidad);
      }
      tiempoUs = micros() - ultimoTiempoUs;
      ultimoTiempoUs = micros();

    } else {
      // modo me fui
      if (ultimoBorde == izquierda) {
        analogWrite(pwmMotorI, 0);
        analogWrite(pwmMotorD, 20);
      } else if (ultimoBorde == derecha) {
        analogWrite(pwmMotorI, 20);
        analogWrite(pwmMotorD, 0);
      }
    }
  }
  esperarReboteBoton();

  // inmediatamente después de presionar el botón para salir del ciclo,
  // se apagan los motores
  apagarMotores();
  
  // hasta que se suelte el botón, espera
  while (apretado(boton1));
  esperarReboteBoton();

}
