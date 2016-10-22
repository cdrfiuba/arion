#include <avr/io.h>
//#include <avr/interrupt.h>
#include <EEPROM.h>

/** inicio de parámetros configurables **/

// parámetro para mostrar información por puerto serie en el ciclo principal
// const bool DEBUG = false;
const bool DEBUG = true;

// parámetros de velocidades máximas en recta y curva
int rangoVelocidadRecta = 120; // velocidad real = rango - freno / 2
int rangoVelocidadCurva = 90;
int rangoVelocidadAfuera = 0;

// velocidad permitida en reversa al aplicar reduccionVelocidad en PID
int velocidadFrenoRecta = 70;
int velocidadFrenoCurva = 90;
int velocidadFrenoAfuera = 0;

// parámetros PID
float kPRecta = 0.03;
float kDRecta = 5.0;
float kPCurva = 0.03;
float kDCurva = 4.0;
//const float kI = 1.0 / 2500.0;

// parámetro para reducir a 0 rangoVelocidad cuando se entra en una curva,
// para frenar de forma controlada
const int tiempoFrenoInicioCurva = 200; //ms

// parámetros encoders
const int cantidadDeSegmentos = 1;
// Arrays donde se guardan las distancias medidas por los encoders
unsigned int distanciasRuedaIzquierda[cantidadDeSegmentos] = {};
unsigned int distanciasRuedaDerecha[cantidadDeSegmentos] = {};
const int aprenderDistancias = 0;
const int usarDistancias = 1;
const int ignorarDistancias = 2;
// determina si se graban los valores en la EEPROM, si se usan para controlar
// la velocidad de recta y curva, o si se ignoran
const int modoUsoDistancias = ignorarDistancias;
const int cantidadDeVueltasADar = 1; // en aprendizaje, se frena al terminar
const int distanciaAnticipoCurva = 300; // medido en cuentas de encoder
bool usarCarrilIzquierdo = false;

// parámetros para usar velocidades distintas en cada recta y en cada curva, 
// de cada carril (izq o der) y parámetros para pasar a una velocidad menor 
// después de cierto tiempo en la recta, según el tramo 
// (nota: se puede agregar una recta más para contemplar la última recta, que 
// si bien es la misma en la que se arranca, puede tener hardcodeado la 
// velocidad máxima, pues no es importante si se cae inmediatamente después de 
// terminar esa recta)
const bool usarTiemposPorRecta = false;
const int cantidadDeRectas = 2; // asume que empieza en recta
const int d1 = 130, d2 = 200, dp = 400, dt = 400; // dn = n maderas de largo
const unsigned int tiempoAMaxVelocidadRecta[cantidadDeRectas] = {
  d1, d1
  //d2, d1, d1, d1, 0/*salida cruz*/, d1, d2, dt/*tunel*/, d2, d1, dp/*puente*/, d1, d1, 
  //65000
};
const bool usarVelocidadPorCurva = false;
const int C = rangoVelocidadCurva;
const int velocidadesCurvaCI[cantidadDeRectas] = {C+00, C+00};
const int velocidadesCurvaCD[cantidadDeRectas] = {};

// parámetros para modo curva
const bool MODO_CURVA_INICIAL = false; // para debuggear si arranca en modo curva o no
const int TOLERANCIA_SENSOR_CURVA = 450; // más de 1024 hace que se ignoren los sensores curva

// parámetros de sensoresLinea cuando estadoActualAdentro == false
const int MAXIMO_SENSORES_LINEA = 2000;
const int MINIMO_SENSORES_LINEA = 4000;

// parámetro medido por tiempoUs para compensar tiempo transcurrido
// entre ciclo y ciclo del PID
const int tiempoCicloReferencia = 1040;//390;

// parámetro batería
// 8.23 V => 847
// 8.00 V => 822
// 8.27 V => 848
// 7.71 v => 791
// 7.50 V => 771 // armado con regla de 3
const int MINIMO_VALOR_BATERIA = 760;
int minimoValorBateria = MINIMO_VALOR_BATERIA; // permite modificarlo en caso de emergencia
const int MAXIMO_VALOR_BATERIA = 859; // = 8.4V / 2 (divisor resistivo) * 1023.0 / 5V

// parámetros para promedio ponderado de sensoresLinea
const int COEFICIENTE_SENSOR_IZQ     = 0;
const int COEFICIENTE_SENSOR_CEN_IZQ = 2000;
const int COEFICIENTE_SENSOR_CEN     = 3000;
const int COEFICIENTE_SENSOR_CEN_DER = 4000;
const int COEFICIENTE_SENSOR_DER     = 6000;
// centro de línea para sensoresLinea
const int centroDeLinea = 3000;

// parámetros para estadoActualAdentro,
// determina si se debe clampear los valores de sensoresLinea en el PID
const int tolerancia = 50; // margen de ruido al medir negro
const int toleranciaBorde = 500; // valor a partir del cual decimos que estamos casi afuera

// parámetro para usar ADC asíncrono, mediante interrupciones
const bool usarInterrupcionesADC = true;

/** fin de parámetros configurables **/


// definición de pines del micro. para Arión 2016 LNR
const int pwmMotorD = 9; // OC1A
const int pwmMotorI = 10; // OC1B
const int sentidoMotorD = 5;
const int sentidoMotorI = 6;
const int led1 = 12;  // Led 1
const int led2 = 13;  // Led 2
const int led3 = 11;  // Led 3
const int boton1 = 8; // Boton1(pcb)   boton3 (code): Calibrar
const int boton2 = 7; // pin 1 arduino  - pin 31(avr)
const int boton3 = 7; // Boton2(pcb)   boton1 (code): Arrancar
const int sensor0 = A0; // sensor0 (code)   sensor2(pcb) izquierda  [izq]
const int sensor1 = A1; // cenIzq           sensor3(pcb)
const int sensor2 = A2; // cen              sensor4(pcb)
const int sensor3 = A3; // cenDer           sensor5(pcb)
const int sensor4 = A4; // der              sensor6(pcb)
const int batteryControl = A6; // 
const int sensorCurvaIzq = A7; //           sensor1(pcb)
const int sensorCurvaDer = A5; //           sensor7(pcb)
const int habilitador = 4; // PD4

// armado de array de sensores y arrays para calibración
const int cantidadDeSensores = 7;
int sensoresSinCalibrar[cantidadDeSensores];
int sensores[cantidadDeSensores];
 // guarda los valores mínimos y máximos usados al calibrar
int minimosSensores[cantidadDeSensores];
int maximosSensores[cantidadDeSensores];
float coeficientesSensores[cantidadDeSensores];
byte canalesADC[cantidadDeSensores] = {
  sensor0, sensor1, sensor2, sensor3, sensor4, sensorCurvaIzq, sensorCurvaDer
};
volatile byte indiceConversion = 0;

// indices de array sensores
const int izq      = 0;
const int cenIzq   = 1;
const int cen      = 2;
const int cenDer   = 3;
const int der      = 4;
const int curvaIzq = 5;
const int curvaDer = 6;

// control para inicializar la calibración de los sensores
// sólo cuando se prende el robot
bool inicializarCalibracionInicial = true;

// dirección motor
const int atras = HIGH;
const int adelante = LOW;

// dirección de direccionMovimientoLateral
const int haciaDerecha = 1;
const int haciaIzquierda = 0;

// bordes de ultimoBorde para modo "me fui"
const int derecha = 1;
const int izquierda = 0;

// contadores de encoders
volatile unsigned int contadorMotorIzquierdo = 0;
volatile unsigned int contadorMotorDerecho = 0;

// macro y string de debug por puerto serie
char debug_string_buffer[30];
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer);

#define clearBit(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define setBit(sfr, bit)   (_SFR_BYTE(sfr) |=  _BV(bit))
#define EEPROM_LENGTH 1024 // el ATmega328 tiene 1KB de EEPROM

#define led1On() digitalWrite(led1, LOW)
#define led2On() digitalWrite(led2, LOW)
#define led3On() digitalWrite(led3, LOW)
#define led1Off() digitalWrite(led1, HIGH)
#define led2Off() digitalWrite(led2, HIGH)
#define led3Off() digitalWrite(led3, HIGH)

#define SERIAL_BPS 115200

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
  //pinMode(sensorCurvaIzq, INPUT);
  //pinMode(sensorCurvaDer, INPUT);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(habilitador, OUTPUT);

  pinMode(boton1, INPUT);
  pinMode(boton2, INPUT);
  pinMode(boton3, INPUT);

  Serial.begin(SERIAL_BPS);

  // pone el prescaler del ADC Clock en 16
  // esto reduce el tiempo de cada conversión AD de ~112us a ~20us
  //setBit(ADCSRA, ADPS2);
  //clearBit(ADCSRA, ADPS1);
  //clearBit(ADCSRA, ADPS0);

  // habilita interrupciones globales
  // NOTA: Arduino deshabilita y habilita interrupciones cuando quiere leer
  // los valores de millis() y micros().
  sei();

  // Configuración de encoders
  // Para poder leer encoders con las interrupciones de Pin Change
  // (que no son externas) y distinguir las dos interrupciones,
  // tienen que estar en grupos distintos de PCIE0, PCIE1 y PCIE2.
  // Actualmente está configurado INT0 (externa) y PCINT22 (pin change)

  // configura INT0 (pin digital 2) en logical change
  // EICRA determina el modo de disparo de la interrupción
  // (en EIFR cambia el bit INTF0 cuando se dispara la interrupción)
  //clearBit(EICRA, ISC01);
  //setBit(EICRA, ISC00);
  //setBit(EIMSK, INT0);
  // alternativamente se lo puede configurar como PCINT, usando
  // PCINT18 en PCIE2 y PCMSK2
  // setBit(PCICR, PCIE2);
  // setBit(PCMSK2, PCINT18);

  // configura PCINT22 (pin digital 6), del grupo PCINT2
  // (en PCIFR cambia el bit PCIF2 cuando se dispara la interrupción)
  //setBit(PCICR, PCIE2);
  //setBit(PCMSK2, PCINT22);

  if (inicializarCalibracionInicial) {
    for (int i = 0; i < cantidadDeSensores; i++) {
      sensoresSinCalibrar[i] = 0;
      sensores[i] = 0;
      minimosSensores[i] = 0;
      maximosSensores[i] = 1023;
      coeficientesSensores[i] = 1.0;
    }
    inicializarCalibracionInicial = false;
  }
  leerCalibracionDeEEPROM();

  digitalWrite(habilitador, HIGH);
  led1Off();
  led2Off();
  led3Off();
  apagarMotores();

  // Sube la frecuencia de PWM, cambiando el prescaler del Timer1.
  // La frecuencia debería pasar de 512Hz a 64Hz.
  // El default es 0b011, y pasaría a 0b100.
  // A mayor frecuencia, menor torque.
  //setBit(TCCR1B, CS12);
  //clearBit(TCCR1B, CS11);
  //clearBit(TCCR1B, CS10);

  // reseteo el valor del encoder
  contadorMotorIzquierdo = 0;
  contadorMotorDerecho = 0;

  // pone en marcha las mediciones continuas de los AD
  if (usarInterrupcionesADC) {
    iniciarConversiones();
  }

}

bool apretado(int boton) {
  return (digitalRead(boton) == LOW);
}

void esperarReboteBoton() {
  delay(5);
}

inline void obtenerSensores() {
  // este procedimiento no se usa cuando las conversiones AD se hacen
  // mediante interrupciones
  if (usarInterrupcionesADC) return;
  // carga en el array de sensores las lecturas AD de cada sensor
  // este proceso lleva 112us con el ADC con prescaler 16
  sensoresSinCalibrar[izq]      = 1024 - analogRead(sensor0);
  sensoresSinCalibrar[cenIzq]   = 1024 - analogRead(sensor1);
  sensoresSinCalibrar[cen]      = 1024 - analogRead(sensor2);
  sensoresSinCalibrar[cenDer]   = 1024 - analogRead(sensor3);
  sensoresSinCalibrar[der]      = 1024 - analogRead(sensor4);
  sensoresSinCalibrar[curvaIzq] = 1024 - analogRead(sensorCurvaIzq);
  sensoresSinCalibrar[curvaDer] = 1024 - analogRead(sensorCurvaDer);
}

inline void obtenerSensoresCalibrados() {
  int valor = 0;
  obtenerSensores();
  for (int i = 0; i < cantidadDeSensores; i++) {
    valor = (sensoresSinCalibrar[i] - minimosSensores[i]);
    valor = valor * coeficientesSensores[i];
    if (valor > 1023) {
      valor = 1023;
    } else if (valor < 0) {
      valor = 0;
    }
    sensores[i] = (int)valor;
  }
}

void calibrarSensores() {
  // leo los sensores, y guardo los mínimos y los máximos
  obtenerSensores();
  for (int i = 0; i < cantidadDeSensores; i++) {
    if (sensoresSinCalibrar[i] < minimosSensores[i]) {
      minimosSensores[i] = sensoresSinCalibrar[i];
    }
    if (sensoresSinCalibrar[i] > maximosSensores[i]) {
      maximosSensores[i] = sensoresSinCalibrar[i];
    }
    coeficientesSensores[i] = 1023.0 / (float)(maximosSensores[i] - minimosSensores[i]);
  }
}

void mostrarSensorLEDs(int sensor) {
  if (sensor >= cantidadDeSensores || sensor < 0) {
    return;
  }
  digitalWrite(led1, ((sensores[sensor] / 768) ? HIGH : LOW));
  digitalWrite(led2, ((sensores[sensor] / 512) ? HIGH : LOW));
  digitalWrite(led3, ((sensores[sensor] / 256) ? HIGH : LOW));
}

void mostrarSensoresPorSerie() {
  debug("%.3d ", analogRead(batteryControl));
  debug("%.4d ", sensores[izq]);
  debug("%.4d ", sensores[cenIzq]);
  debug("%.4d ", sensores[cen]);
  debug("%.4d ", sensores[cenDer]);
  debug("%.4d ", sensores[der]);
  debug("%.4d ", sensores[curvaIzq]);
  debug("%.4d ", sensores[curvaDer]);
  if (modoUsoDistancias != ignorarDistancias) {
    debug("%.4d ", contadorMotorIzquierdo);
    debug("%.4d ", contadorMotorDerecho);
  }
  if (modoUsoDistancias == usarDistancias) {
    debug("%s ", "|");
    for (int i = 0; i < cantidadDeSegmentos; i++) {
      debug("{%d, ", distanciasRuedaIzquierda[i]);
      debug("%d} ", distanciasRuedaDerecha[i]);
    }
  }
  debug("%s", "\n");
}

void apagarMotores() {
  digitalWrite(sentidoMotorI, adelante);
  digitalWrite(sentidoMotorD, adelante);
  analogWrite(pwmMotorI, 0);
  analogWrite(pwmMotorD, 0);
}

inline void chequearBateria() {
  if (analogRead(batteryControl) < minimoValorBateria) {
    led1On();
    led2On();
    led3On();
  } else {
    led1Off();
    led2Off();
    led3Off();
  }
}

inline void chequearBateriaBloqueante() {
  if (analogRead(batteryControl) < minimoValorBateria) {
    // si la batería está por debajo del mínimo, parpadea LEDs
    // hasta que se aprieta el botón
    while (!apretado(boton2)) {
      led1On();
      led2On();
      led3On();
      delay(200);
      led1Off();
      led2Off();
      led3Off();
      delay(200);
    }
    esperarReboteBoton();

    // luego de apretar el botón
    minimoValorBateria -= 10;

    led1Off();
    led2Off();
    led3Off();

    // hasta que se suelte el botón, espera
    while (apretado(boton2));
    esperarReboteBoton();
  }
}

#define readString(data, a, b, c) (data[0] == a && data[1] == b && data[2] == c)
void leerVariablesDeSerie() {
  char b[] = {0, 0, 0};
  int nuevoValorInt = 0;
  float nuevoValorFloat = 0.0;
  if (Serial.available()) {
    b[0] = Serial.read();
    b[1] = Serial.read();
    b[2] = Serial.read();
    Serial.setTimeout(10); // timeout para parseInt
    if (b[0] == 0 || b[1] == 0 || b[2] == 0) {
      // no hacer nada
      
    // Rango Velocidad, Recta Curva Afuera
    } else if (readString(b, 'r', 'v', 'r')) {
      nuevoValorInt = Serial.parseInt();
      Serial.print("rangoVelocidadRecta = ");
      rangoVelocidadRecta = nuevoValorInt;
      Serial.println(nuevoValorInt);
      delay(2000);
    } else if (readString(b, 'r', 'v', 'c')) {
      nuevoValorInt = Serial.parseInt();
      Serial.print("rangoVelocidadCurva = ");
      rangoVelocidadCurva = nuevoValorInt;
      Serial.println(nuevoValorInt);
      delay(2000);
    } else if (readString(b, 'r', 'v', 'a')) {
      nuevoValorInt = Serial.parseInt();
      Serial.print("rangoVelocidadAfuera = ");
      rangoVelocidadAfuera = nuevoValorInt;
      Serial.println(nuevoValorInt);
      delay(2000);

    // Velocidad Freno, Recta Curva Afuera
    } else if (readString(b, 'v', 'f', 'r')) {
      nuevoValorInt = Serial.parseInt();
      Serial.print("velocidadFrenoRecta = ");
      velocidadFrenoRecta = nuevoValorInt;
      Serial.println(nuevoValorInt);
      delay(2000);
    } else if (readString(b, 'v', 'f', 'c')) {
      nuevoValorInt = Serial.parseInt();
      Serial.print("velocidadFrenoCurva = ");
      velocidadFrenoCurva = nuevoValorInt;
      Serial.println(nuevoValorInt);
      delay(2000);
    } else if (readString(b, 'v', 'f', 'a')) {
      nuevoValorInt = Serial.parseInt();
      Serial.print("velocidadFrenoAfuera = ");
      velocidadFrenoAfuera = nuevoValorInt;
      Serial.println(nuevoValorInt);
      delay(2000);

    // k P Recta, P Curva, D Recta, D Curva
    } else if (readString(b, 'k', 'p', 'r')) {
      nuevoValorFloat = Serial.parseFloat();
      Serial.print("kPRecta = ");
      kPRecta = nuevoValorFloat;
      Serial.println(nuevoValorFloat);
      delay(2000);
    } else if (readString(b, 'k', 'd', 'r')) {
      nuevoValorFloat = Serial.parseFloat();
      Serial.print("kDRecta = ");
      kDRecta = nuevoValorFloat;
      Serial.println(nuevoValorFloat);
      delay(2000);
    } else if (readString(b, 'k', 'p', 'c')) {
      nuevoValorFloat = Serial.parseFloat();
      Serial.print("kPCurva = ");
      kPCurva = nuevoValorFloat;
      Serial.println(nuevoValorFloat);
      delay(2000);
    } else if (readString(b, 'k', 'd', 'c')) {
      nuevoValorFloat = Serial.parseFloat();
      Serial.print("kDCurva = ");
      kDCurva = nuevoValorFloat;
      Serial.println(nuevoValorFloat);
      delay(2000);

    // k P Recta, P Curva, D Recta, D Curva
    } else if (readString(b, 'd', 'l', 'd')) {
      Serial.println("te canto los datos:");
      // rangos velocidad
      debug("rangoVelocidadRecta = %d\n", rangoVelocidadRecta);
      debug("rangoVelocidadCurva = %d\n", rangoVelocidadCurva);
      debug("rangoVelocidadAfuera = %d\n", rangoVelocidadAfuera);

      // velocidades freno
      debug("velocidadFrenoRecta = %d\n", velocidadFrenoRecta);
      debug("velocidadFrenoCurva = %d\n", velocidadFrenoCurva);
      debug("velocidadFrenoAfuera = %d\n", velocidadFrenoAfuera);

      // los floats no andan con sprintf, por lo que uso println
      debug("%s", "kPRecta = "); Serial.println(kPRecta);
      debug("%s", "kDRecta = "); Serial.println(kDRecta);
      debug("%s", "kPCurva = "); Serial.println(kPCurva);
      debug("%s", "kDCurva = "); Serial.println(kDCurva);
      delay(5000);
    }
  }
}

void loop() {
  // definición de variables locales
  float errP = 0;
  float errPAnterior = 0;
  //float errI = 0;
  float errD = 0;
  float kP = 1;
  float kD = 0;
  int rangoVelocidad;
  int velocidadFreno;
  int reduccionVelocidad;
  int velocidadMotorFrenado;
  int direccionMovimientoLateral;
  int sensoresLinea = 0;
  bool estadoActualAdentro = true;
  int sensorCurvaIzqActivo;
  int sensorCurvaDerActivo;
  bool calibracionReseteada = false;
  int ultimoBorde = izquierda;
  bool modoCurva = MODO_CURVA_INICIAL;
  // para calcular tiempo entre ciclos de PID.
  int tiempoUs = tiempoCicloReferencia; // no debe ser 0, pues se usa para dividir
  unsigned long int ultimoTiempoUs = 0; // guarda el valor de micros()
  unsigned long int ultimoTiempoRecta = 0; // guarda el valor de millis()
  unsigned long int ultimoTiempoCurva = 0; // guarda el valor de millis()
  int contadorRecta = 0;
  int velocidadesCurvaPorTramo[cantidadDeRectas];
  int indiceSegmento = 0; // almacena el indice de segmento de la pista
  int distanciaActual = 0;
  int distanciaEsperada = 0;
  int cantidadDeVueltasRestantes = cantidadDeVueltasADar;
  bool cambioModoCurvaCompletado = true;
  byte contadorDataSerie = 0;

  // si fue seleccionado el modo usarVelocidadPorCurva,
  // precargo la data del carril seleccionado
  if (usarVelocidadPorCurva) {
    if (usarCarrilIzquierdo) {
      for (int i = 0; i < cantidadDeRectas; i++) {
        velocidadesCurvaPorTramo[i] = velocidadesCurvaCI[i];
      }
    } else {
      for (int i = 0; i < cantidadDeRectas; i++) {
        velocidadesCurvaPorTramo[i] = velocidadesCurvaCD[i];
      }
    }
  }

  // inicialización de todas las cosas
  setup();

  // hasta que se presione el botón, espera
  while (!apretado(boton1)) {
    leerVariablesDeSerie();
    
    // deshabilito temporalmente las interrupciones para poder hacer lecturas
    // manuales con AnalogRead de la batería
    noInterrupts();
    delayMicroseconds(120);
    chequearBateriaBloqueante();
    obtenerSensoresCalibrados();
    mostrarSensoresPorSerie();
    interrupts();
    // le doy tiempo a las interrupciones ADC
    // a refrescar el valor de los sensores
    delay(1);

    // carga opcional de información de encoders,
    // que depende del carril actual
    if (modoUsoDistancias == usarDistancias) {
      leerDistanciasDeEEPROM();
    }

    // muestra estado de encoders
    if (usarCarrilIzquierdo) {
      led1On();
      led2Off();
    } else {
      led1Off();
      led2On();
    }

    // calibración usando el botón
    calibracionReseteada = false;
    while (apretado(boton3)) {
      if (!calibracionReseteada) {
        // reseteo la calibración
        for (int i = 0; i < cantidadDeSensores; i++) {
          minimosSensores[i] = 1023;
          maximosSensores[i] = 0;
        }
        calibracionReseteada = true;
        
        // reuso la bandera de calibracionReseteada para que esto se ejecute
        // una sola vez por apretada de botón
        usarCarrilIzquierdo = !usarCarrilIzquierdo;
      }

      led1On();
      calibrarSensores();
      led1Off();
      led2Off();
      led3Off();
      delay(50);
    }
    if (calibracionReseteada) {
      guardarCalibracionEnEEPROM();
    }

  }
  esperarReboteBoton();
  led1Off();
  led2Off();
  led3Off();

  // hasta que se suelte el botón, espera
  while (apretado(boton1));
  esperarReboteBoton();

  // seteo de rangoVelocidad para arranque gradual
  if (modoCurva) {
    rangoVelocidad = rangoVelocidadCurva;
  } else {
    rangoVelocidad = rangoVelocidadRecta;
  }

  // arranque gradual
  for (int i = 0; i < rangoVelocidad / 10; i++) {
    analogWrite(pwmMotorD, i * 10);
    analogWrite(pwmMotorI, i * 10);
    delay(10);
  }

  // inicialización tiempos
  ultimoTiempoRecta = millis();
  ultimoTiempoCurva = millis();
  ultimoTiempoUs = micros();

  // ejecuta el ciclo principal hasta que se presione el botón
  while (!apretado(boton1)) {

    //chequearBateria();
    obtenerSensoresCalibrados();

    if (sensores[izq] > toleranciaBorde) {
      ultimoBorde = izquierda;
    } else if (sensores[der] > toleranciaBorde) {
      ultimoBorde = derecha;
    }

    // si me fui, entro en modo "corrección máxima"
      estadoActualAdentro = true;
    if (sensores[cenIzq] < tolerancia &&
        sensores[cen]    < tolerancia &&
        sensores[cenDer] < tolerancia) {
      if ( (sensores[izq] < tolerancia && sensores[der] < toleranciaBorde) ||
           (sensores[izq] < toleranciaBorde && sensores[der] < tolerancia) ) {
        estadoActualAdentro = false;
      }
    }

    // 50 microsegundos
    // modo pid
    // linea = (0 * s0 + 1000 * s1 + 2000 * s2 + 3000 * s3 + 4000 * s4) / (s0 + s1 + s2 + s3 + s4)
    sensoresLinea = (
      (long)sensores[izq]    * COEFICIENTE_SENSOR_IZQ +
      (long)sensores[cenIzq] * COEFICIENTE_SENSOR_CEN_IZQ +
      (long)sensores[cen]    * COEFICIENTE_SENSOR_CEN +
      (long)sensores[cenDer] * COEFICIENTE_SENSOR_CEN_DER +
      (long)sensores[der]    * COEFICIENTE_SENSOR_DER
    ) / (
      (long)sensores[izq]    +
      (long)sensores[cenIzq] +
      (long)sensores[cen]    +
      (long)sensores[cenDer] +
      (long)sensores[der]
    );

    // clampea valor extremo para indicarle al PID
    // que corrija con toda su fuerza
    if (estadoActualAdentro == false) {
      if (ultimoBorde == izquierda) {
        sensoresLinea = MINIMO_SENSORES_LINEA;
      } else {
        sensoresLinea = MAXIMO_SENSORES_LINEA;
      }
    }

    sensorCurvaIzqActivo = ((sensores[curvaIzq] > TOLERANCIA_SENSOR_CURVA) ? 1 : 0);
    sensorCurvaDerActivo = ((sensores[curvaDer] > TOLERANCIA_SENSOR_CURVA) ? 1 : 0);
    
    if (sensorCurvaIzqActivo == 0 && sensorCurvaDerActivo == 0) {
      cambioModoCurvaCompletado = true;
    }
    if (sensorCurvaIzqActivo == 1 && sensorCurvaDerActivo == 1 && cambioModoCurvaCompletado) {
      cambioModoCurvaCompletado = false;
      // tengo seguridad de que pasó el rebote del sensor
      modoCurva = !modoCurva;

      if (modoUsoDistancias == aprenderDistancias) {
        distanciasRuedaIzquierda[indiceSegmento] = contadorMotorIzquierdo;
        distanciasRuedaDerecha[indiceSegmento] = contadorMotorDerecho;
      }

      // indice usado para identificar el segmento
      indiceSegmento = (indiceSegmento + 1) % cantidadDeSegmentos;

      // Reseteo el valor del encoder
      contadorMotorIzquierdo = 0;
      contadorMotorDerecho = 0;

      // en el modo aprender, se apaga cuando termina la cantidad de vueltas
      if (modoUsoDistancias == aprenderDistancias) {
        if (indiceSegmento == 0) {
          cantidadDeVueltasRestantes--;
          if (cantidadDeVueltasRestantes <= 0) {
            apagarMotores();
            guardarDistanciasEnEEPROM();
            break;
          }
        }
      }

      // si paso a modo curva, freno porque venia rápido
      if (modoCurva) {
        ultimoTiempoCurva = millis();
      } else {
        ultimoTiempoRecta = millis();
        contadorRecta++;
        if (contadorRecta == cantidadDeRectas) {
          contadorRecta = 0;
        }
      }
    }

    if (modoCurva) {
      kP = kPCurva;
      kD = kDCurva;
      rangoVelocidad = rangoVelocidadCurva;
      velocidadFreno = velocidadFrenoCurva;
      if (usarVelocidadPorCurva) {
        rangoVelocidad = velocidadesCurvaPorTramo[contadorRecta];
      }
      if (millis() - ultimoTiempoCurva < tiempoFrenoInicioCurva) {
        rangoVelocidad = 0;
        velocidadFreno = 255;
      }
      led2Off();
      led3Off();
    } else {
      kP = kPRecta;
      kD = kDRecta;
      rangoVelocidad = rangoVelocidadRecta;
      velocidadFreno = velocidadFrenoRecta;
      if (usarTiemposPorRecta) {
        if (millis() - ultimoTiempoRecta > tiempoAMaxVelocidadRecta[contadorRecta]) {
          kP = kPCurva;
          kD = kDCurva;
          rangoVelocidad = rangoVelocidadCurva;
          velocidadFreno = velocidadFrenoCurva;
          
          // velocidades manuales según segmento, después de frenar
          // if (contadorRecta == 3) {
          //   rangoVelocidad = 20;
          // }
          led3On();
        } else {
          led3Off();
        }
      }
      
      if (modoUsoDistancias == usarDistancias) {
        distanciaActual = (contadorMotorIzquierdo + contadorMotorDerecho) / 2;
        distanciaEsperada = (distanciasRuedaIzquierda[indiceSegmento] + distanciasRuedaDerecha[indiceSegmento]) / 2;
        if (distanciaActual + distanciaAnticipoCurva > distanciaEsperada) {
          rangoVelocidad = rangoVelocidadAfuera; // velocidad Afuera es más lenta
          //velocidadFreno = velocidadFrenoRecta; // con freno curva cabecea mucho
          led3On();
        } else {
          led3Off();
        }
      }
      led2On();
    }

    if (estadoActualAdentro == false) {
      rangoVelocidad = rangoVelocidadAfuera;
      velocidadFreno = velocidadFrenoAfuera;
      led1On();
    } else {
      led1Off();
    }

    // 20 microsegundos
    errP = sensoresLinea - centroDeLinea;
    // errI += errP * tiempoCicloReferencia / tiempoUs;
    errD = (errP - errPAnterior) * tiempoUs / tiempoCicloReferencia;
    errPAnterior = errP;
    // reduccionVelocidad = errP * kP + errD * kD + errI * kI;
    reduccionVelocidad = errP * kP + errD * kD;

    // constrain
    if (reduccionVelocidad < -rangoVelocidad - velocidadFreno) {
      reduccionVelocidad = -rangoVelocidad - velocidadFreno;
    } else if (reduccionVelocidad > rangoVelocidad + velocidadFreno) {
      reduccionVelocidad = rangoVelocidad + velocidadFreno;
    }

    if (reduccionVelocidad < 0) {
      direccionMovimientoLateral = haciaIzquierda;
    } else {
      direccionMovimientoLateral = haciaDerecha;
    }

    reduccionVelocidad = abs(reduccionVelocidad);
    velocidadMotorFrenado = abs(rangoVelocidad - reduccionVelocidad);

    if (direccionMovimientoLateral == haciaIzquierda) {
      // si la reducción es mayor al rango de velocidad,
      // uno de los motores va para atrás
      if (reduccionVelocidad > rangoVelocidad) {
        digitalWrite(sentidoMotorI, atras);
        digitalWrite(sentidoMotorD, adelante);
        analogWrite(pwmMotorI, 255 - velocidadMotorFrenado);
        analogWrite(pwmMotorD, rangoVelocidad);
      } else {
        digitalWrite(sentidoMotorI, adelante);
        digitalWrite(sentidoMotorD, adelante);
        analogWrite(pwmMotorI, velocidadMotorFrenado);
        analogWrite(pwmMotorD, rangoVelocidad);
      }
    } else if (direccionMovimientoLateral == haciaDerecha) {
      // si la reducción es mayor al rango de velocidad,
      // uno de los motores va para atrás
      if (reduccionVelocidad > rangoVelocidad) {
        digitalWrite(sentidoMotorI, adelante);
        digitalWrite(sentidoMotorD, atras);
        analogWrite(pwmMotorI, rangoVelocidad);
        analogWrite(pwmMotorD, 255 - velocidadMotorFrenado);
      } else {
        digitalWrite(sentidoMotorI, adelante);
        digitalWrite(sentidoMotorD, adelante);
        analogWrite(pwmMotorI, rangoVelocidad);
        analogWrite(pwmMotorD, velocidadMotorFrenado);
      }
    }

    if (DEBUG) {
      // Permite ver por puerto serie cuánto tarda el ciclo de PID
      // antes de perder tiempo mandando cosas por puerto serie.
      // Usado para medir tiempoCicloReferencia.
      //tiempoUs = micros() - ultimoTiempoUs;
      //debug("%.4i ", sensoresLinea);
      if (contadorDataSerie == 0) debug("%.4i ", sensoresLinea);
      Serial.print("\n");
      contadorDataSerie++;
      if (contadorDataSerie == 1) {
        contadorDataSerie = 0;
      }
    
    }
    // mide el tiempo entre ciclo y ciclo, necesario para calcular errD y errI
    tiempoUs = micros() - ultimoTiempoUs;

    // espera a que se cumpla el tiempoCicloReferencia
    if (tiempoUs < tiempoCicloReferencia) {
      delayMicroseconds(tiempoCicloReferencia - tiempoUs);
      tiempoUs = micros() - ultimoTiempoUs;
    }

    ultimoTiempoUs = micros();

  }
  esperarReboteBoton();

  // inmediatamente después de presionar el botón para salir del ciclo,
  // se apagan los motores
  apagarMotores();

  // hasta que se suelte el botón, espera
  while (apretado(boton1));
  esperarReboteBoton();

}

//// handler para PCINT22
//ISR(PCINT2_vect) {
  //contadorMotorIzquierdo++;
//}
//// handler para INT0
//ISR(INT0_vect) {
  //contadorMotorDerecho++;
//}

void guardarIntEnEEPROM(int valor, int posicion) {
  uint8_t low = valor & 0xFF;
  uint8_t high = valor >> 8;
  EEPROM.write(posicion, low);
  EEPROM.write(posicion + 1, high);
}
int leerIntDeEEPROM(int posicion) {
  uint8_t low;
  uint8_t high;
  low = EEPROM.read(posicion);
  high = EEPROM.read(posicion + 1);
  return (high << 8) + low;
}

void guardarDistanciasEnEEPROM() {
  int posicion = 0;

  // las distancias del carril derecho se guardan después de todas las del
  // carril izquierdo
  if (!usarCarrilIzquierdo) {
    // cada segmento usa 2 bytes por rueda
    posicion = cantidadDeSegmentos * 4;
  }

  for (int i = 0; i < cantidadDeSegmentos; i++) {
    guardarIntEnEEPROM(distanciasRuedaIzquierda[i], posicion);
    posicion = posicion + 2;
    guardarIntEnEEPROM(distanciasRuedaDerecha[i], posicion);
    posicion = posicion + 2;
  }
}
void leerDistanciasDeEEPROM() {
  int posicion = 0;

  // las distancias del carril derecho se guardan después de todas las del
  // carril izquierdo
  if (!usarCarrilIzquierdo) {
    // cada segmento usa 2 bytes por rueda
    posicion = cantidadDeSegmentos * 4;
  }

  for (int i = 0; i < cantidadDeSegmentos; i++) {
    distanciasRuedaIzquierda[i] = leerIntDeEEPROM(posicion);
    posicion = posicion + 2;
    distanciasRuedaDerecha[i] = leerIntDeEEPROM(posicion);
    posicion = posicion + 2;
  }
}

void guardarCalibracionEnEEPROM() {
  // cada segmento usa 2 bytes por rueda, por la cantidad de segmentos,
  // para los dos carriles
  int posicion = cantidadDeSegmentos * 2 * 4;
  
  // leo los sensores, y guardo los mínimos y los máximos
  for (int i = 0; i < cantidadDeSensores; i++) {
    guardarIntEnEEPROM(minimosSensores[i], posicion);
    posicion = posicion + 2;
    guardarIntEnEEPROM(maximosSensores[i], posicion);
    posicion = posicion + 2;
  }
}
void leerCalibracionDeEEPROM() {
  // cada segmento usa 2 bytes por rueda, por la cantidad de segmentos,
  // para los dos carriles
  int posicion = cantidadDeSegmentos * 2 * 4;

  // leo los sensores, y guardo los mínimos y los máximos
  for (int i = 0; i < cantidadDeSensores; i++) {
    minimosSensores[i] = leerIntDeEEPROM(posicion);
    posicion = posicion + 2;
    maximosSensores[i] = leerIntDeEEPROM(posicion);
    posicion = posicion + 2;
    coeficientesSensores[i] = 1023.0 / (float)(maximosSensores[i] - minimosSensores[i]);
  }
}

void iniciarConversiones() {
  byte pin;
  for (byte i = 0; i < cantidadDeSensores; i++) {
    pin = canalesADC[i];
    // Arduino pin conversion
    #if defined(__AVR_ATmega32U4__)
      if (pin >= 18) pin -= 18; // allow for channel or pin numbers
      pin = analogPinToChannel(pin);
    #else
      if (pin >= 14) pin -= 14; // allow for channel or pin numbers
    #endif
    canalesADC[i] = pin;
  }  
  pin = canalesADC[indiceConversion];
  
  #if defined(ADCSRB) && defined(MUX5)
    // the MUX5 bit of ADCSRB selects whether we're reading from channels
    // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
  #endif

  // when ADLAR is low, ADCL must be read first, otherwise, reading ADCH is
  // enough (if 8 bits are sufficient)
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (pin & 0x07);

  // interrupt enable, start the conversion
  ADCSRA |= (1 << ADIE) | (1 << ADSC);
}

ISR(ADC_vect) {
  sensoresSinCalibrar[indiceConversion] = 1024 - ADC;
  if (++indiceConversion == cantidadDeSensores) {
    indiceConversion = 0;
  }
  
  byte pin = canalesADC[indiceConversion];
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (pin & 0x07);
  ADCSRA |= _BV(ADSC); // Start next conversion  
}
