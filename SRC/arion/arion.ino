// nada marron nada blanco rojo nada
//              micro


// nada nada blanco rojo marron
//            usb

char debug_string_buffer[50];
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer); \
  delay(1);
  
const bool DEBUG = false;

const int atras = HIGH;
const int adelante = LOW;
int velocidadMotorFrenado;
const int haciaIzquierda = 0;
const int haciaDerecha = 1;
int direccionMovimientoLateral;

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

// para batería
// 8.23 V => 847 
// 8.00 V => 822
// 7.50 V  => 771 
const int MINIMO_VALOR_BATERIA = 771;

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


const int tolerancia = 0; // Margen de ruido al medir negro.
const int toleranciaBorde = 200; // Mínimo para decidir cuál fue el último borde

const int velocidadFreno = 20;
// velocidadMinima + rangoVelocidad <= 255 (o explota)
//const int velocidadMinima = 10;
const int rangoVelocidad = 150;
int reduccionVelocidad;
int errP;
int errPAnterior;
int errI;
int errD;

int sensoresLinea = 0;
const int centroDeLinea = 3000;
const int coeficienteErrorPmult = 1;
const int coeficienteErrorPdiv = 7;
const int coeficienteErrorIdiv = 2500;
const int coeficienteErrorDmult = 19;
const int coeficienteErrorDdiv = 1;

bool estadoActualAdentro; // determina si se usa modo PID o modo "me fui"
  // bordes para modo "me fui"
const int derecha = 1;
const int izquierda = 0;
int ultimoBorde;

// para calcular tiempo entre ciclos de PID.
// no debe ser 0, pues se usa para dividir
long int ultimoTiempoUs = 0; // guarda el valor de micros()
int tiempoUs = 0; // guarda el tiempo del ciclo
const int tiempoCicloReferencia = 840;

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
  digitalWrite(sentidoMotorI, adelante);
  digitalWrite(sentidoMotorD, adelante);
  
  errP = 0;
  errI = 0;
  errD = 0;
  errPAnterior = 0;
  ultimoBorde = izquierda;
  estadoActualAdentro = true;
  tiempoUs = 1.0;
  ultimoTiempoUs = 0;
  
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
    if (sensores[i] > maximosSensores[i]) {
      maximosSensores[i] = sensores[i];
    }
  }
}

void mostrarSensorLEDs(int sensor) {
  if ((sensor >= cantidadDeSensores) || (sensor < 0)) {
    return; 
  }
  digitalWrite(led1, ((sensores[sensor] / 768) ? HIGH : LOW));
  digitalWrite(led2, ((sensores[sensor] /  512) ? HIGH : LOW));
  digitalWrite(led3, ((sensores[sensor] /  256) ? HIGH : LOW));
}

void mostrarSensores() {
  debug("%.4d ", analogRead(batteryControl));
  debug("%.4d ", sensores[izq]);
  debug("%.4d ", sensores[cenIzq]);
  debug("%.4d ", sensores[cen]);
  debug("%.4d ", sensores[cenDer]);
  debug("%.4d\n", sensores[der]);
}

void apagarMotores() {
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

  for (int i = 0; i < rangoVelocidad / 10; i++) {
    analogWrite(pwmMotorD, i * 10);
    analogWrite(pwmMotorI, i * 10);
    delay(10);
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
      // modo pid
      // linea = (0 * s0 + 1000 * s1 + 2000 * s2 + 3000 * s3 + 4000 * s4) / (s0 + s1 + s2 + s3 + s4)
      // 0 a 4000, donde 2000 es el centroDeLinea
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
  
      errP = sensoresLinea - centroDeLinea;
      errI = errP * tiempoCicloReferencia / tiempoUs;
      errD = (errP - errPAnterior) * tiempoUs / tiempoCicloReferencia;
      // err_i += (err_p >> 8);
      //if ( (err_i >= VALOR_MAX_INT16 - VALOR_MAX_ERR_P) || (err_i <= -(VALOR_MAX_INT16 - VALOR_MAX_ERR_P)) ) {
      //    err_i -= (err_p >> 8);
      //}
      errPAnterior = errP;
      //delayMicroseconds (2000);
      reduccionVelocidad = (errP * coeficienteErrorPmult) / coeficienteErrorPdiv  + (errD * coeficienteErrorDmult) / coeficienteErrorDdiv + errI / coeficienteErrorIdiv;
      
      // errP va entre -2000 y 2000, con p=1/12 reduccionVelocidad va entre -166 y +166 
      // errD va entre -4000 y 4000, con d=1/30 reduccionVelocidad va entre -133 y +133
      // // err_i toma valores entre -32k y 32k, por lo que su aporte a diff_potencia esta acotado entre -32 y +32 (-32 y +32 para 6 sensores)
      // // err_d toma valores entre -5k y 5k, por lo que su aporte a diff_potencia esta acotado entre -inf y +inf (para los niveles de representacion que manejamos). 
      // // Para un caso normal, en que err_p varie 30 entre una medicion y la siguiente, estará acotado entre -45 y +45
      
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
  
  // debug("%f", tiempoUs);
  // debug("%i\n", ultimoTiempoUs);
  
  // inmediatamente después de presionar el botón para salir del ciclo, 
  // se apagan los motores
  apagarMotores();

  // hasta que se suelte el botón, espera
  while (apretado(boton1));
  esperarReboteBoton();

}
