#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include <PID_v1.h>

/** inicio de parámetros configurables **/
/* Pista larga modificada 19.33 segundos
comentario: curva con oscilaciones
speedSetpoint = 55;
rangoVelocidadRecta = 105;
kPRecta = 0.02;
kDRecta = 1.0;
kPRectaLenta = kPRecta ;
kDRectaLenta = kDRecta;
kPCurva = 0.1;
kDCurva = 1.1;
*/
// parámetro para mostrar información por puerto serie en el ciclo principal
const bool DEBUG = false;
// const bool DEBUG = true;
int speedSetpoint = 55; // cuentas por (ENCODER_ARRAY_SIZE * ENCODER_SUBSAMPLING) ciclos
int cant_curvas = 0;


struct DebugArray {
  double k_p1;
  double k_i1;
  double k_d1;
  double setpoint1;
  double input1;
  double output1;
  int sensoresLinea;
};

// parámetros para estadoActualAdentro,
// determina si se debe clampear los valores de sensoresLinea en el PID
const int tolerancia = 50; // margen de ruido al medir negro
const int toleranciaBorde = 500; // valor a partir del cual decimos que estamos casi afuera

int rangoVelocidad;

// parámetros de velocidades máximas en recta y curva
// 770: 230, 110, 170, 50
// 839: 210, 100, 155, 45
// 859: 200, 95, 150, 45
int rangoVelocidadAprender = 40;
int rangoVelocidadRecta = 105;//85;//95;// 75 (terminó) 128; // velocidad real = rango - freno / 2
int rangoVelocidadRectaLenta = speedSetpoint;//95;
int rangoVelocidadCurva = speedSetpoint;//130;
int rangoVelocidadAfuera = speedSetpoint; //

// velocidad permitida en reversa al aplicar reduccionVelocidad en PID
const int velocidadFrenoRecta = 0; //255;
const int velocidadFrenoCurva = 0; //60;
const int velocidadFrenoAfuera = 0;

// parámetros PID

const float kPRecta = 0.02;//0.02;//0.03;// 0.04;     //1.0 / 9.0; //1/12
const float kDRecta = 1.0;//1 estable pero con probable sobre corrección;//0.125;//075;//;3;        //3.0; //7
const float kPRectaLenta = kPRecta ;// 1.0 / 7.0;
const float kDRectaLenta = kDRecta;//3.0; // 200
const float kPCurva = 0.1;//1.0 / 9.0;
const float kDCurva = 1.1;//3.0; //30
//const float kI = 1.0 / 2500.0;

// parámetros encoders
const int cantidadDeSegmentos = 21;
// Arrays donde se guardan las distancias medidas por los encoders
unsigned int distanciasRuedaIzquierda[cantidadDeSegmentos] = {};
unsigned int distanciasRuedaDerecha[cantidadDeSegmentos] = {};
const int aprenderDistancias = 0;
const int usarDistancias = 1;
const int ignorarDistancias = 2;
// determina si se graban los valores en la EEPROM, si se usan para controlar
// la velocidad de recta y curva, o si se ignoran
const int modoUsoDistancias = usarDistancias;
const int cantidadDeVueltasADar = 1; // en aprendizaje, se frena al terminar
const int distanciaAnticipoCurva = 100; // medido en cuentas de encoder
bool usarCarrilIzquierdo = false;

// parámetros para usar velocidades distintas en cada recta y en cada curva, de cada carril (izq o der)
// y parámetros para pasar a una velocidad menor después de cierto tiempo en la recta, según el tramo
// (nota: se puede agregar una recta más para contemplar la última recta, que si bien es la misma
// en la que se arranca, puede tener hardcodeado la velocidad máxima, pues no es importante si
// se cae inmediatamente después de terminar esa recta)
const bool usarTiemposPorRecta = false;
const int cantidadDeRectas = 25; // asume que empieza en recta
const unsigned int tiempoAMaxVelocidadRecta[cantidadDeRectas] = {0,
  0, 2000, 0, 2000, 2000, 0, 2000, 0, 0, 1000, 1000, 0,    /* 1 vuelta */
  0, 2000, 0, 2000, 2000, 0, 2000, 0, 0, 1000, 1000, 65000 /* 1 vuelta */
};
const bool usarVelocidadPorTramo = false;
const int R = rangoVelocidadRecta;
const int C = rangoVelocidadCurva;
const int velocidadesCurvaCI[cantidadDeRectas] = {C+00, C+00, C+00, C+00};
const int velocidadesCurvaCD[cantidadDeRectas] = {C+00, C+00, C+00, C+00};

// parámetros para modo curva
const bool MODO_CURVA_INICIAL = false; // para debuggear si arranca en modo curva o no
const int TOLERANCIA_SENSOR_CURVA = 600; // más de 1024 hace que se ignore el sensorCurva
const int DEBOUNCE_MODO_CURVA = 10; // ms

// parámetros de sensoresLinea cuando estadoActualAdentro == false
const int MAXIMO_SENSORES_LINEA = 4000;
const int MINIMO_SENSORES_LINEA = 2000;

// parámetros de frenarMotores
const int VELOCIDAD_FRENO_POR_CAMBIO_MODO_CURVA = 255; // 0 apagado, 255 full speed
const int DELAY_FRENO_POR_CAMBIO_MODO_CURVA = 40; // ms

// parámetro medido por tiempoUs para compensar tiempo transcurrido
// entre ciclo y ciclo del PID
const int tiempoCicloReferencia = 1200;//1040;//390;

// parámetro batería
// 8.23 V => 847
// 8.00 V => 822
// 8.27 V => 848
// 7.71 v => 791
// 7.50 V => 771 // armado con regla de 3
const int MINIMO_VALOR_BATERIA = 760;
int minimoValorBateria = MINIMO_VALOR_BATERIA; // permite modificarlo en caso de emergencia
const bool usarTensionCompensadaBateria = false;
const int MAXIMO_VALOR_BATERIA = 859; // = 8.4V / 2 (divisor resistivo) * 1023.0 / 5V

// parámetros para promedio ponderado de sensoresLinea
const int COEFICIENTE_SENSOR_IZQ     = 0;
const int COEFICIENTE_SENSOR_CEN_IZQ = 1000;
const int COEFICIENTE_SENSOR_CEN     = 2000;
const int COEFICIENTE_SENSOR_CEN_DER = 3000;
const int COEFICIENTE_SENSOR_DER     = 4000;
// centro de línea para sensoresLinea
const int centroDeLinea = 2000;

// Encoder circular buffer storage for speed estimation
#define ENCODER_ARRAY_SIZE 61
#define ENCODER_SUBSAMPLING 1

// Both motor PIDs
#define MOTOR_LIMIT_OUTPUT 255
#define MOTOR_PID_SAMPLETIME 1

// PID motor 1
#define KP_MOTOR1 4.0
#define KI_MOTOR1 0.0
#define KD_MOTOR1 0.0005
// PID motor 2
#define KP_MOTOR2 4.0
#define KI_MOTOR2 0.0
#define KD_MOTOR2 0.0005

#define DEBUG_ARRAY_SIZE 20
DebugArray debug_data[DEBUG_ARRAY_SIZE];

/** fin de parámetros configurables **/

////////////////////////////////////////////////////////////////////////////////

// PID motores
const int motorLimit = MOTOR_LIMIT_OUTPUT;
// PID motor 1
double k_p1 = KP_MOTOR1, k_i1 = KI_MOTOR1, k_d1 = KD_MOTOR1;
double setpoint1 = 0, input1 = 0, output1;
unsigned int aux_input1 = 0;
PID motor1Pid(&input1, &output1, &setpoint1, k_p1, k_i1, k_d1, DIRECT);

// PID motor 2
double k_p2 = KP_MOTOR2, k_i2 = KI_MOTOR2, k_d2 = KD_MOTOR2;
double setpoint2 = 0, input2 = 0, output2;
unsigned int aux_input2 = 0;
PID motor2Pid(&input2, &output2, &setpoint2, k_p2, k_i2, k_d2, DIRECT);

// definición de pines del micro.
const int pwmMotorD = 11;
const int pwmMotorI = 10;
const int sentidoMotorD = 3;
const int sentidoMotorI = 5;
const int ledArduino = 13;
const int led1 = 9;
const int led2 = 8;
const int led3 = 12;
const int boton1 = 7;
const int boton2 = 4; // es el pin 6, ahora usado por un encoder
const int boton3 = 4;
const int sensor0 = A0;
const int sensor1 = A1;
const int sensor2 = A2;
const int sensor3 = A3;
const int sensor4 = A5;
const int batteryControl = A4;
const int sensorCurva = A6;

// armado de array de sensores y arrays para calibración
const int cantidadDeSensores = 6;
int sensores[cantidadDeSensores];
 // guarda los valores mínimos y máximos usados al calibrar
int minimosSensores[cantidadDeSensores];
int maximosSensores[cantidadDeSensores];
float coeficientesSensores[cantidadDeSensores];

// indices de array sensores
const int izq    = 0;
const int cenIzq = 1;
const int cen    = 2;
const int cenDer = 3;
const int der    = 4;
const int curva  = 5;

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
unsigned int contadorMotorIzquierdoAnterior = 0;
unsigned int contadorMotorDerechoAnterior = 0;
unsigned int contadorMotorIzquierdoCicloAnterior = 0;
unsigned int contadorMotorDerechoCicloAnterior = 0;


unsigned int contadorMotorIzquierdoArray[ENCODER_ARRAY_SIZE] = {};
unsigned int contadorMotorDerechoArray[ENCODER_ARRAY_SIZE] = {};
byte pBegin = 0; // begin pointer of the circular buffer
byte pEnd = 0;   // end pointer of the circular buffer
unsigned int num_ciclo_programa = 0;
unsigned int num_ciclo_subsampling = 0;

// macro y string de debug por puerto serie
char debug_string_buffer[20];
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer);

#define clearBit(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define setBit(sfr, bit)   (_SFR_BYTE(sfr) |=  _BV(bit))
#define EEPROM_LENGTH 1024 // el ATmega328 tiene 1KB de EEPROM

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
  pinMode(sensorCurva, INPUT);

  pinMode(ledArduino, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  pinMode(boton1, INPUT);
  pinMode(boton2, INPUT);
  pinMode(boton3, INPUT);

  Serial.begin(230400);

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
  clearBit(EICRA, ISC01);
  setBit(EICRA, ISC00);
  setBit(EIMSK, INT0);
  // alternativamente se lo puede configurar como PCINT, usando
  // PCINT18 en PCIE2 y PCMSK2
  // setBit(PCICR, PCIE2);
  // setBit(PCMSK2, PCINT18);

  // configura PCINT22 (pin digital 6), del grupo PCINT2
  // (en PCIFR cambia el bit PCIF2 cuando se dispara la interrupción)
  setBit(PCICR, PCIE2);
  setBit(PCMSK2, PCINT22);

  if (inicializarCalibracionInicial) {
    for (int i = 0; i < cantidadDeSensores; i++) {
      sensores[i] = 0;
      minimosSensores[i] = 0;
      maximosSensores[i] = 1023;
      coeficientesSensores[i] = 1.0;
    }
    inicializarCalibracionInicial = false;
  }
  leerCalibracionDeEEPROM();


  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  digitalWrite(ledArduino, LOW);
  apagarMotores();

}

bool apretado(int boton) {
  return (digitalRead(boton) == LOW);
}

void esperarReboteBoton() {
  delay(5);
}

inline void obtenerSensores() {
  // carga en el array de sensores las lecturas AD de cada sensor
  // este proceso lleva 112us con el ADC con prescaler 16
  sensores[izq]    = 1024 - analogRead(sensor0);
  sensores[cenIzq] = 1024 - analogRead(sensor1);
  sensores[cen]    = 1024 - analogRead(sensor2);
  sensores[cenDer] = 1024 - analogRead(sensor3);
  sensores[der]    = 1024 - analogRead(sensor4);
  sensores[curva]  = 1024 - analogRead(sensorCurva);
}

inline void obtenerSensoresCalibrados() {
  int valor = 0;
  obtenerSensores();
  for (int i = 0; i < cantidadDeSensores; i++) {
    valor = (sensores[i] - minimosSensores[i]);
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
    if (sensores[i] < minimosSensores[i]) {
      minimosSensores[i] = sensores[i];
    }
    if (sensores[i] > maximosSensores[i]) {
      maximosSensores[i] = sensores[i];
    }
    coeficientesSensores[i] = 1023.0 / (float)(maximosSensores[i] - minimosSensores[i]);
  }
}

void mostrarSensorLEDs(int sensor) {
  if ((sensor >= cantidadDeSensores) || (sensor < 0)) {
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
  debug("%.4d ", sensores[curva]);
  debug("%.4d ", contadorMotorIzquierdo);
  debug("%.4d ", contadorMotorDerecho);
  debug("%.4d ", rangoVelocidad);
  if (modoUsoDistancias == usarDistancias) {
    debug("%s ", "|");
    for (int i = 0; i < cantidadDeSegmentos; i++) {
      debug("{%d, ", distanciasRuedaIzquierda[i]);
      debug("%d} ", distanciasRuedaDerecha[i]);
    }
  }
  debug("%s", "\n");
  // debug("%s", "\n--\n");
  // debug("%d\n", num_ciclo_programa % DEBUG_ARRAY_SIZE);
  // for (int i = 0; i < DEBUG_ARRAY_SIZE; i++)
  // {
  //   Serial.print(debug_data[i].k_p1);
  //   Serial.print(" ");
  //   Serial.print(debug_data[i].k_i1);
  //   Serial.print(" ");
  //   Serial.print(debug_data[i].k_d1);
  //   Serial.print(" ");
  //   Serial.print(debug_data[i].setpoint1);
  //   Serial.print(" ");
  //   Serial.print(debug_data[i].input1);
  //   Serial.print(" ");
  //   Serial.print(debug_data[i].output1);
  //   Serial.print(" ");
  //   debug("%.4d\n", debug_data[i].sensoresLinea);
  // }
  // debug("%s", "\n!!\n");

}

void apagarMotores() {
  digitalWrite(sentidoMotorI, adelante);
  digitalWrite(sentidoMotorD, adelante);
  analogWrite(pwmMotorI, 0);
  analogWrite(pwmMotorD, 0);
}

inline void chequearBateria() {
  if (analogRead(batteryControl) < minimoValorBateria) {
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
  if (analogRead(batteryControl) < minimoValorBateria) {
    // si la batería está por debajo del mínimo, parpadea LEDs
    // hasta que se aprieta el botón
    while (!apretado(boton2)) {
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

    // luego de apretar el botón
    minimoValorBateria -= 10;

    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);

    // hasta que se suelte el botón, espera
    while (apretado(boton2));
    esperarReboteBoton();
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
  // int rangoVelocidad;
  int velocidadFreno;
  int errorTotal;
  int reduccionVelocidad;
  int velocidadMotorFrenado;
  int direccionMovimientoLateral;
  int sensoresLinea = 0;
  bool estadoActualAdentro = true;
  bool ultimoEstadoActualAdentro = true;
  int sensorCurvaActivo;
  int ultimoValorSensorCurva = 0;
  bool calibracionReseteada = false;
  int ultimoBorde = izquierda;
  bool modoCurva = MODO_CURVA_INICIAL;
  // para calcular tiempo entre ciclos de PID.
  int tiempoUs = tiempoCicloReferencia; // no debe ser 0, pues se usa para dividir
  unsigned long int ultimoTiempoUs = 0; // guarda el valor de micros()
  unsigned long int ultimoTiempoModoCurva = 0; // guarda el valor de millis()
  unsigned long int ultimoTiempoRecta = 0; // guarda el valor de millis()
  int contadorRecta = 0;
  int velocidadesCurvaPorTramo[cantidadDeRectas];
  float coeficienteBateria;
  int indiceSegmento = 0; // almacena el indice de segmento de la pista
  int distanciaActual = 0;
  int distanciaEsperada = 0;
  int cantidadDeVueltasRestantes = cantidadDeVueltasADar;

  // Holt Linear Exponential Smoothing
  // General parameters for both wheels
  float alpha = 0.035;
  float beta = 0.1;
  float kForecast = 1;
  float alphaComplement = 1 - alpha;
  float betaComplement = 1 - beta;
  // Left Wheel
  float LtIzq = 0;
  float LtIzqAnterior = 0;
  float TrendIzq = 0;
  float TrendIzqAnterior = 0;
  float YtIzq, YIzqPredictivo;
  // Right Wheel
  float LtDer = 0;
  float LtDerAnterior = 0;
  float TrendDer = 0;
  float TrendDerAnterior = 0;
  float YtDer, YDerPredictivo;


  motor1Pid.SetMode(AUTOMATIC);
  motor1Pid.SetSampleTime(MOTOR_PID_SAMPLETIME); // ms
  motor1Pid.SetTunings(k_p1, k_i1, k_d1);
  motor1Pid.SetOutputLimits( -motorLimit, motorLimit);


  motor2Pid.SetMode(AUTOMATIC);
  motor2Pid.SetSampleTime(MOTOR_PID_SAMPLETIME); // ms
  motor2Pid.SetTunings(k_p2, k_i2, k_d2);
  motor2Pid.SetOutputLimits( -motorLimit, motorLimit);


  // si fue seleccionado el modo usarVelocidadPorTramo,
  // precargo la data del carril seleccionado
  if (usarVelocidadPorTramo) {
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
  if (modoUsoDistancias == aprenderDistancias)
  {
    rangoVelocidadRecta      = rangoVelocidadAprender;
    rangoVelocidadRectaLenta = rangoVelocidadAprender;
    rangoVelocidadCurva      = rangoVelocidadAprender;
    rangoVelocidadAfuera     = rangoVelocidadAprender;
  }
  setup();
  cant_curvas = 0;
  // hasta que se presione el botón, espera
  while (!apretado(boton1)) {
    chequearBateriaBloqueante();

    obtenerSensoresCalibrados();
    mostrarSensoresPorSerie();
    // mostrarSensorLEDs(cen);

    // carga opcional de información de encoders
    if (modoUsoDistancias == usarDistancias) {
      leerDistanciasDeEEPROM();
    }

    // muestra carril elegido (usado con encoders)
    if (usarCarrilIzquierdo) {
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
    } else {
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
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
        // usarCarrilIzquierdo = !usarCarrilIzquierdo;
      }

      digitalWrite(led1, HIGH);
      calibrarSensores();
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      delay(50);
    }
    if (calibracionReseteada) {
      guardarCalibracionEnEEPROM();
    }

  }
  esperarReboteBoton();
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  digitalWrite(ledArduino, LOW);

  // hasta que se suelte el botón, espera
  while (apretado(boton1));
  esperarReboteBoton();

  // calculo el coeficiente de la batería según la carga que tenga ahora
  if (usarTensionCompensadaBateria) {
    coeficienteBateria = MAXIMO_VALOR_BATERIA / analogRead(batteryControl);
  } else {
    coeficienteBateria = 1.0;
  }

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
  ultimoTiempoUs = micros();

  // Re inicializacion
  contadorMotorIzquierdo = 0;
  contadorMotorDerecho   = 0;
  contadorMotorIzquierdoAnterior = 0;
  contadorMotorDerechoAnterior   = 0;
  pBegin = 0; // begin pointer of the circular buffer
  pEnd = 0;   // end pointer of the circular buffer
  num_ciclo_programa = 0;
  num_ciclo_subsampling = 0;
  for ( int i = 0; i < ENCODER_ARRAY_SIZE ; i++ )
  {
    contadorMotorIzquierdoArray[i] = 0;
    contadorMotorDerechoArray[i] = 0;
  }

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
    if ((sensores[cenIzq] < tolerancia) &&
        (sensores[cen]    < tolerancia) &&
        (sensores[cenDer] < tolerancia)) {
      if ( ((sensores[izq]  < tolerancia) && (sensores[der] < toleranciaBorde) ) ||
           ((sensores[izq]  < toleranciaBorde) && (sensores[der] < tolerancia) ) ) {
        estadoActualAdentro = false;
        //digitalWrite(led3, HIGH);
        if (estadoActualAdentro != ultimoEstadoActualAdentro) {
          //frenarMotores();
        }
      } else {
        //digitalWrite(led3, LOW);
        estadoActualAdentro = true;
      }

    } else {
      estadoActualAdentro = true;
      //digitalWrite(led3, LOW);
    }
    ultimoEstadoActualAdentro = estadoActualAdentro;

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

    // NM 20160925 remove noise from ADC, last 2 bits are BS
    // sensoresLinea = ((sensoresLinea >> 2) << 2);

    // clampea valor extremo para indicarle al PID
    // que corrija con toda su fuerza
    if (estadoActualAdentro == false) {
      if (ultimoBorde == izquierda) {
        sensoresLinea = MINIMO_SENSORES_LINEA;
      } else {
        sensoresLinea = MAXIMO_SENSORES_LINEA;
      }
    }

    sensorCurvaActivo = ((sensores[curva] > TOLERANCIA_SENSOR_CURVA) ? 1 : 0);
    if (sensorCurvaActivo == 1 && sensorCurvaActivo != ultimoValorSensorCurva) {
      if (millis() - ultimoTiempoModoCurva > DEBOUNCE_MODO_CURVA) {
        // tengo seguridad de que pasó el rebote del sensor

        // Prueba NM
        // cant_curvas++;
        // if (cant_curvas == 25)
        //   rangoVelocidad = 30;

        modoCurva = !modoCurva;

        if (modoUsoDistancias == aprenderDistancias) {
          distanciasRuedaIzquierda[indiceSegmento] = contadorMotorIzquierdo - contadorMotorIzquierdoAnterior;
          distanciasRuedaDerecha[indiceSegmento] = contadorMotorDerecho - contadorMotorDerechoAnterior;
        }

        // indice usado para identificar el segmento
        indiceSegmento = (indiceSegmento + 1) % cantidadDeSegmentos;

        // Reseteo el valor del encoder
        contadorMotorIzquierdoAnterior = contadorMotorIzquierdo;
        contadorMotorDerechoAnterior = contadorMotorDerecho;

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

        ultimoTiempoModoCurva = millis();
        // si paso a modo curva, freno porque venia rápido
        if (modoCurva) {
          // frenarMotores(); // Para las pruebas de encoders lo comento
        } else {
          ultimoTiempoRecta = millis();
          contadorRecta++;
          if (contadorRecta == cantidadDeRectas) {
            contadorRecta = 0;
          }
        }
      }
    }
    ultimoValorSensorCurva = sensorCurvaActivo;

    if (modoCurva) {
      kP = kPCurva;
      kD = kDCurva;
      rangoVelocidad = rangoVelocidadCurva;
      velocidadFreno = velocidadFrenoCurva;
      if (usarVelocidadPorTramo) {
        rangoVelocidad = velocidadesCurvaPorTramo[contadorRecta];
      }
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
    } else {
      kP = kPRecta;
      kD = kDRecta;
      rangoVelocidad = rangoVelocidadRecta;
      velocidadFreno = velocidadFrenoRecta;
      if (usarTiemposPorRecta) {
        if (millis() - ultimoTiempoRecta > tiempoAMaxVelocidadRecta[contadorRecta]) {
          rangoVelocidad = rangoVelocidadCurva;
          digitalWrite(led3, HIGH);
        } else {
          digitalWrite(led3, LOW);
        }
      }
      if (modoUsoDistancias == usarDistancias) {
        // velocidades manuales según segmento, respetando distancia freno
        //if (indiceSegmento == 2) { // puente
          //rangoVelocidad = 160;
        //}

        distanciaEsperada = (distanciasRuedaIzquierda[indiceSegmento] + distanciasRuedaDerecha[indiceSegmento]) / 2;
        distanciaActual = (contadorMotorIzquierdo - contadorMotorIzquierdoAnterior)/2 + (contadorMotorDerecho - contadorMotorDerechoAnterior) / 2;
        if (distanciaActual + distanciaAnticipoCurva > distanciaEsperada) {
            rangoVelocidad = rangoVelocidadRectaLenta;
            kP = kPRectaLenta;
            kD = kDRectaLenta;
          //velocidadFreno = velocidadFrenoRecta; // con freno curva cabecea mucho
          digitalWrite(led3, HIGH);
        } else {
          digitalWrite(led3, LOW);
        }
        // velocidades manuales según segmento, sin respetar distancia freno
        //if (indiceSegmento == 8) { // primer tramo corto
          //rangoVelocidad = rangoVelocidadCurva;
        //}
        //if (indiceSegmento == 12) { // segundo tramo corto
          //rangoVelocidad = rangoVelocidadCurva;
        //}
      }
      digitalWrite(led2, HIGH);
    }

    if (estadoActualAdentro == false) {
      rangoVelocidad = rangoVelocidadAfuera;
      velocidadFreno = velocidadFrenoAfuera;
      digitalWrite(led1, HIGH);
    } else {
      digitalWrite(led1, LOW);
    }

    // aplico el coeficiente de compensación de tensión de la batería
    if (usarTensionCompensadaBateria) {
      rangoVelocidad = rangoVelocidad * coeficienteBateria;
      velocidadFreno = velocidadFreno * coeficienteBateria;
      if (rangoVelocidad > 255) {
        rangoVelocidad = 255;
      }
      if (velocidadFreno > 255) {
        velocidadFreno = 255;
      }
    }

    // 20 microsegundos
    errP = sensoresLinea - centroDeLinea;
    // errI += errP * tiempoCicloReferencia / tiempoUs;
    errD = (errP - errPAnterior) * tiempoUs / tiempoCicloReferencia;
    errPAnterior = errP;
    // reduccionVelocidad = errP * kP + errD * kD + errI * kI;
    errorTotal = errP * kP + errD * kD;
    reduccionVelocidad = errorTotal;

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
      setpoint1 = velocidadMotorFrenado;
      setpoint2 = rangoVelocidad;

    } else {
      setpoint1 = rangoVelocidad;
      setpoint2 = velocidadMotorFrenado;

    }


    num_ciclo_programa++;
    /*
    if (  num_ciclo_programa % ENCODER_SUBSAMPLING == 0 )
    {
      pBegin = (ENCODER_ARRAY_SIZE + num_ciclo_subsampling ) % ENCODER_ARRAY_SIZE;
      pEnd   = (ENCODER_ARRAY_SIZE + num_ciclo_subsampling + 1 ) % ENCODER_ARRAY_SIZE;
      contadorMotorIzquierdoArray[pBegin] = contadorMotorIzquierdo;
      contadorMotorDerechoArray[pBegin]   = contadorMotorDerecho;
      aux_input1 = contadorMotorIzquierdoArray[pBegin] - contadorMotorIzquierdoArray[pEnd];
      aux_input2 = contadorMotorDerechoArray[pBegin]   - contadorMotorDerechoArray[pEnd];
      input1 = aux_input1;
      input2 = aux_input2;
      num_ciclo_subsampling++;
    }
    */


    // Cantidad de pasos entre ciclos
    YtIzq = contadorMotorIzquierdo - contadorMotorIzquierdoCicloAnterior;
    YtDer = contadorMotorDerecho   - contadorMotorDerechoCicloAnterior;
    contadorMotorIzquierdoCicloAnterior = contadorMotorIzquierdo;
    contadorMotorDerechoCicloAnterior = contadorMotorDerecho;

    // Level Updating Equation
    LtIzq = alpha * YtIzq + alphaComplement * (LtIzqAnterior + TrendIzqAnterior);
    LtDer = alpha * YtDer + alphaComplement * (LtDerAnterior + TrendDerAnterior);
    // Trend Updating Equation
    TrendIzq = beta * (LtIzq - LtIzqAnterior) + betaComplement * TrendIzqAnterior;
    TrendDer = beta * (LtDer - LtDerAnterior) + betaComplement * TrendDerAnterior;
    // Forecasting Equation
    YIzqPredictivo = (LtIzq + kForecast*TrendIzq)*61; // 61 is just a scale factor
    YDerPredictivo = (LtDer + kForecast*TrendDer)*61; // 61 is just a scale factor
    // Previous values storage
    TrendIzqAnterior = TrendIzq;
    LtIzqAnterior = LtIzq;
    TrendDerAnterior = TrendDer;
    LtDerAnterior = LtDer;
    // Clamping
    if (YIzqPredictivo < 0 )
      YIzqPredictivo = 0;
    if (YDerPredictivo < 0 )
      YDerPredictivo = 0;

    input1 = YIzqPredictivo;
    input2 = YDerPredictivo;

    motor1Pid.Compute();
    motor2Pid.Compute();

    // output1 = 50;
    // output2 = 50;

    if (output1 < 0)
    {
      digitalWrite(sentidoMotorI, atras);
      analogWrite(pwmMotorI, 255 + (int) output1 );
      // analogWrite(pwmMotorI, 255 );
    }
    else
    {
      digitalWrite(sentidoMotorI, adelante);
      analogWrite(pwmMotorI, (int) output1);
    }

    if (output2 < 0)
    {
      digitalWrite(sentidoMotorD, atras);
      analogWrite(pwmMotorD, 255 + (int) output2 );
      // analogWrite(pwmMotorD, 255 );
    }
    else
    {
      digitalWrite(sentidoMotorD, adelante);
      analogWrite(pwmMotorD, (int) output2);
    }


    /* Debugging last values of PID for bizarre behaviour
      byte aux_idx = num_ciclo_programa % DEBUG_ARRAY_SIZE;
      debug_data[aux_idx].k_p1 = motor1Pid.GetKp();
      debug_data[aux_idx].k_i1 = motor1Pid.GetKi();
      debug_data[aux_idx].k_d1 = motor1Pid.GetKd();
      debug_data[aux_idx].setpoint1 = setpoint1;
      debug_data[aux_idx].input1 = input1;
      debug_data[aux_idx].output1 = output1;
      debug_data[aux_idx].sensoresLinea = sensoresLinea;
    */



    if (DEBUG) {
      // Permite ver por puerto serie cuánto tarda el ciclo de PID
      // antes de perder tiempo mandando cosas por puerto serie.
      // Usado para medir tiempoCicloReferencia.
      tiempoUs = micros() - ultimoTiempoUs;
      debug("%.4i ", tiempoUs);
      // debug("%4d ", contadorMotorIzquierdo);
      // debug("%4d ", contadorMotorDerecho);
      // debug("%4d ", num_ciclo_programa);
      debug("%.2i ", (int) YtIzq);
      debug("%.2i ", (int) YtDer);

      debug("%u ", aux_input1);
      debug("%u ", aux_input2);
      Serial.print( YIzqPredictivo);
      Serial.print(" ");
      Serial.print( YDerPredictivo);
      // debug("%u ", aux_input2);
      // debug("%3d\n", rangoVelocidad);

      // debug("%u ", sensoresLinea);
      // Serial.print(" ");
      // Serial.print( setpoint1);
      // Serial.print(" ");
      // Serial.print( setpoint2);
      // Serial.print(" ");
      // Serial.print( output1);
      // Serial.print(" ");
      // Serial.print(output2);
      Serial.print("\n");

      // Serial.print(" ");
      // Serial.print( motor1Pid.GetKp());
      // Serial.print(" ");
      // Serial.print( motor1Pid.GetKi());
      // Serial.print(" ");
      // Serial.print( motor1Pid.GetKd());


      // debug("%d ", pBegin);
      // debug("%d ", pEnd);
      // debug("% .4i ", (int)(errP * kP));
      // debug("% .4i ", (int)(errD * kD));
      // debug("%.4i ", abs(errorTotal));
      // debug("%.4i ", velocidadMotorFrenado);
      // debug("%s", "\n");
    }
    // mide el tiempo entre ciclo y ciclo, necesario para calcular errD y errI
    tiempoUs = micros() - ultimoTiempoUs;

    // Time keeping loop
    while (tiempoUs < tiempoCicloReferencia) {
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

inline void frenarMotores() {
  digitalWrite(led1, HIGH);

  // pone los motores para atrás a la velocidad del parámetro y espera
  digitalWrite(sentidoMotorI, atras);
  digitalWrite(sentidoMotorD, atras);
  analogWrite(pwmMotorI, 255 - VELOCIDAD_FRENO_POR_CAMBIO_MODO_CURVA);
  analogWrite(pwmMotorD, 255 - VELOCIDAD_FRENO_POR_CAMBIO_MODO_CURVA);

  delay(DELAY_FRENO_POR_CAMBIO_MODO_CURVA); // ms

  // pone los motores para adelante, frenados
  digitalWrite(sentidoMotorI, adelante);
  digitalWrite(sentidoMotorD, adelante);
  analogWrite(pwmMotorI, 0);
  analogWrite(pwmMotorD, 0);

  digitalWrite(led1, LOW);
}

// handler para PCINT22
ISR(PCINT2_vect) {
  contadorMotorIzquierdo++;
}
// handler para INT0
ISR(INT0_vect) {
  contadorMotorDerecho++;
}

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
