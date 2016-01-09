/** inicio de parámetros configurables **/

// parámetro para mostrar información por puerto serie en el ciclo principal
const bool DEBUG = false;

// parámetros para estadoActualAdentro,
// determina si se debe clampear los valores de sensoresLinea en el PID
const int tolerancia = 50; // margen de ruido al medir negro
const int toleranciaBorde = 500; // valor a partir del cual decimos que estamos casi afuera

// parámetros de velocidades máximas
const int rangoVelocidad = 255;
const int rangoVelocidadAfuera = 50;

// velocidad permitida en reversa al aplicar reduccionVelocidad en PID
const int velocidadFreno = 0;

// parámetros PID
const float kP = 1.0 / 7.0;
const float kD = 35.0;
//const float kI = 1.0 / 2500.0;

// parámetros de sensoresLinea cuando estadoActualAdentro == false
const int MAXIMO_SENSORES_LINEA = 4000;
const int MINIMO_SENSORES_LINEA = 2000;

// parámetro medido por tiempoUs para compensar tiempo transcurrido
// entre ciclo y ciclo del PID
const int tiempoCicloReferencia = 1040;

// parámetro batería
// 8.23 V => 847
// 8.00 V => 822
// 8.27 V => 848
// 7.71 v => 791
// 7.50 V => 771 // armado con regla de 3
const int MINIMO_VALOR_BATERIA = 760;
const bool usarTensionCompensadaBateria = false;
const int MAXIMO_VALOR_BATERIA = 859;// = 8.4V / 2 (divisor resitivo) * 1023.0 / 5V


// parámetros para promedio ponderado de sensoresLinea
const int COEFICIENTE_SENSOR_IZQ     = 0;
const int COEFICIENTE_SENSOR_CEN_IZQ = 2000;
const int COEFICIENTE_SENSOR_CEN     = 3000;
const int COEFICIENTE_SENSOR_CEN_DER = 4000;
const int COEFICIENTE_SENSOR_DER     = 6000;
// centro de línea para sensoresLinea
const int centroDeLinea = 3000;

/** fin de parámetros configurables **/


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
const int boton2 = 6;
const int boton3 = 4;
const int sensor0 = A0;
const int sensor1 = A1;
const int sensor2 = A2;
const int sensor3 = A3;
const int sensor4 = A5;
const int batteryControl = A4;

// armado de array de sensores y arrays para calibración
const int cantidadDeSensores = 5;
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

// macro y string de debug por puerto serie
char debug_string_buffer[20];
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer);

#define clearBit(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define setBit(sfr, bit)   (_SFR_BYTE(sfr) |=  _BV(bit))

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

  Serial.begin(115200);

  // pone el prescaler del ADC Clock en 16
  // esto reduce el tiempo de cada conversión AD de ~112us a ~20us
  //setBit(ADCSRA, ADPS2);
  //clearBit(ADCSRA, ADPS1);
  //clearBit(ADCSRA, ADPS0);

  if (inicializarCalibracionInicial) {
    for (int i = 0; i < cantidadDeSensores; i++) {
      sensores[i] = 0;
      minimosSensores[i] = 0;
      maximosSensores[i] = 1023;
      coeficientesSensores[i] = 1.0;
    }
    inicializarCalibracionInicial = false;
  }
  
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

void mostrarSensores() {
  debug("%.3d ", analogRead(batteryControl));
  debug("%.4d ", sensores[izq]);
  debug("%.4d ", sensores[cenIzq]);
  debug("%.4d ", sensores[cen]);
  debug("%.4d ", sensores[cenDer]);
  debug("%.4d\n", sensores[der]);
}

void apagarMotores() {
  digitalWrite(sentidoMotorI, adelante);
  digitalWrite(sentidoMotorD, adelante);
  analogWrite(pwmMotorD, 0);
  analogWrite(pwmMotorI, 0);
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
    // si la batería está por debajo del mínimo, parpadea LEDs
    // hasta que se apreta el botón
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
  int rangoVelocidad;
  int velocidadFreno;
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
  setup();

  // hasta que se presione el botón, espera,
  // y muestra en los leds el valor del sensor central
  while (!apretado(boton1)) {
    obtenerSensoresCalibrados();
    mostrarSensorLEDs(cen);
    mostrarSensores();
    chequearBateriaBloqueante();
    
    while (apretado(boton3)) {
      if (!calibracionReseteada) {
        // reseteo la calibración
        for (int i = 0; i < cantidadDeSensores; i++) {
          minimosSensores[i] = 1023;
          maximosSensores[i] = 0;
        }
        calibracionReseteada = true;
      }

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
  digitalWrite(ledArduino, LOW);
  
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

  // calculo el coeficiente de la batería según la carga que tenga ahora
  if (usarTensionCompensadaBateria) {
    coeficienteBateria = MAXIMO_VALOR_BATERIA / analogRead(batteryControl);
  } else {
    coeficienteBateria = 1.0;
  }
  
  // inicializacion tiempos
  ultimoTiempoRecta = millis();
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

    // clampea valor extremo para indicarle al PID 
    // que corrija con toda su fuerza
    if (estadoActualAdentro == false) {
      if (ultimoBorde == izquierda) {
        sensoresLinea = MINIMO_SENSORES_LINEA;
      } else {
        sensoresLinea = MAXIMO_SENSORES_LINEA;
      }
    }

    if (modoCurva) {
      rangoVelocidad = rangoVelocidadCurva;
      velocidadFreno = velocidadFrenoCurva;
      if (usarVelocidadPorTramo) {
        rangoVelocidad = velocidadesCurvaPorTramo[contadorRecta];
      }
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
    } else {
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
      digitalWrite(led2, HIGH);
    }
    
    if (estadoActualAdentro == false) {
      rangoVelocidad = rangoVelocidadAfuera;
    }

    // aplico el coeficiente de compensacion de tensión de la batería
    rangoVelocidad = rangoVelocidad * coeficienteBateria;
    velocidadFreno = velocidadFreno * coeficienteBateria;
    if (rangoVelocidad > 255) {
      rangoVelocidad = 255;
    }
    if (velocidadFreno > 255) {
      velocidadFreno = 255;
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
      tiempoUs = micros() - ultimoTiempoUs;
      debug("%.4i ", tiempoUs);
      debug("% .5i ", (int)errP);
      debug("% .5i ", (int)errD);
      debug("%.4i\n", reduccionVelocidad);
    }
    // mide el tiempo entre ciclo y ciclo, necesario para calcular errD y errI
    tiempoUs = micros() - ultimoTiempoUs;
    
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

