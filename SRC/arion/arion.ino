// nada marron nada blanco rojo nada
//              micro

// nada nada blanco rojo marron
//            usb

char debug_string_buffer[50];
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer); \
  delay(1);

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
const int sensor4 = A4;
const int batteryControl = A5;

const int cantidadDeSensores = 5;
int sensores[cantidadDeSensores];

// indices de array sensores
const int izq    = 0;
const int cenIzq = 1;
const int cen    = 2;
const int cenDer = 3;
const int der    = 4;

const int tolerancia = 5; // Margen de ruido al medir negro.
const int toleranciaBorde = 50; // Mínimo para decidir cuál fue el último borde

// velocidadMinima + rangoVelocidad <= 255 (o explota)
const int velocidadMinima = 0;
const int rangoVelocidad = 20;
int reduccionVelocidad;
int errP;
int errPAnterior;
int errI;
int errD;

int sensoresLinea = 0;
const int centroDeLinea = 2000;
const int coeficienteErrorP = 12;
const int coeficienteErrorI = 6000;
const int coeficienteErrorD = 3;

boolean estadoActualAdentro; // determina si se usa modo PID o modo "me fui"
// bordes para modo "me fui"
const int derecha = 1;
const int izquierda = 0;
int ultimoBorde;

// tiempo entre ciclos de PID.
// no debe ser 0, pues se usa para dividir
long tiempoUs = 1;

void setup() {                
  // como los motores se manejan con AnalogWrite, 
  // no hace falta ponerlos como salida
  // [Yo lo haria, simplemente para documentar y hacer codigo mas explicity. Explicit better than implicit - Lucas ]
  // pinMode(pwmMotorD, OUTPUT);
  // pinMode(pwmMotorI, OUTPUT);

  pinMode(sentidoMotorD, OUTPUT); 
  pinMode(sentidoMotorI, OUTPUT); 
  
  // como los sensores y el batteryControl se leen con AnalogRead, 
  // no hace falta ponerlos como entrada
  // pinMode(sensor0, INPUT);
  // pinMode(sensor1, INPUT);
  // pinMode(sensor2, INPUT);
  // pinMode(sensor3, INPUT);
  // pinMode(sensor4, INPUT);
  // pinMode(batteryControl, INPUT);
  
  pinMode(ledArduino, OUTPUT);
  pinMode(led1, OUTPUT);  
  pinMode(led2, OUTPUT);  
  pinMode(led3, OUTPUT);

  pinMode(boton1, INPUT);
  pinMode(boton2, INPUT);
  pinMode(boton3, INPUT);

  Serial.begin(9600);

  for (int i = 0; i < cantidadDeSensores; i++) {
    sensores[i] = 0;
  }
  
  apagarMotores();
  
  errP = 0;
  errI = 0;
  errD = 0;
  errPAnterior = 0;
  ultimoBorde = izquierda;
  estadoActualAdentro = true;
  tiempoUs = 1;
  
}

boolean apretado(int boton) {
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

void mostrarSensorLEDs(int sensor) {
  if ((sensor >= cantidadDeSensores) || (sensor < 0)) {
    return; 
  }
  digitalWrite(led1, ((sensores[sensor] / 1024) ? HIGH : LOW));
  digitalWrite(led2, ((sensores[sensor] /  512) ? HIGH : LOW));
  digitalWrite(led3, ((sensores[sensor] /  256) ? HIGH : LOW));
}

void mostrarSensores() {
  //char buffer[50];
  // printf(buffer, "%.4d %.4d %.4d %.4d %.4d", sensores[izq], sensores[cenIzq], sensores[cen], sensores[cenDer], sensores[der]);
  // Serial.println(buffer);
  debug("%.4d ", sensores[izq]);
  debug("%.4d ", sensores[cenIzq]);
  debug("%.4d ", sensores[cen]);
  debug("%.4d ", sensores[cenDer]);
  debug("%.4d\n", sensores[der]);
  /*
  Serial.print(sensores[izq]);
  Serial.print(" ");
  Serial.print(sensores[cenIzq]);
  Serial.print(" ");
  Serial.print(sensores[cen]);
  Serial.print(" ");
  Serial.print(sensores[cenDer]);
  Serial.print(" ");
  Serial.print(sensores[der]);
  Serial.println("");
  */
}

void apagarMotores() {
  analogWrite(pwmMotorD, 0);
  analogWrite(pwmMotorI, 0);
}

void loop() {

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
  // hasta que se presione el botón, espera,
  // y muestra en los leds el valor del sensor central
  while (!apretado(boton1)) {
    obtenerSensores();
    mostrarSensorLEDs(cen);
    mostrarSensores();
  }
  esperarReboteBoton();
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  
  // inicialización de todo
  setup();

  // hasta que se suelte el botón, espera 
  while (apretado(boton1));
  esperarReboteBoton();

  // arranque progresivo, de 0 a 250 en 75 ms
  for (int i = 1; i <= 25; i++) {
    analogWrite(pwmMotorD, i * 10);
    analogWrite(pwmMotorI, i * 10);
    delay(3);
  }

  // ejecuta el ciclo principal hasta que se presione el botón
  while (!apretado(boton1)) {
    obtenerSensores();
    
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
        (long)sensores[cenIzq] * 1000 + 
        (long)sensores[cen]    * 2000 + 
        (long)sensores[cenDer] * 3000 + 
        (long)sensores[der]    * 4000
      ) / (
        (long)sensores[izq]    + 
        (long)sensores[cenIzq] + 
        (long)sensores[cen]    + 
        (long)sensores[cenDer] + 
        (long)sensores[der]
      );
  
      errP = sensoresLinea - centroDeLinea;
      errI = errP * tiempoUs;
      errD = (errP - errPAnterior) / tiempoUs;
      // err_i += (err_p >> 8);
      //if ( (err_i >= VALOR_MAX_INT16 - VALOR_MAX_ERR_P) || (err_i <= -(VALOR_MAX_INT16 - VALOR_MAX_ERR_P)) ) {
      //    err_i -= (err_p >> 8);
      //}
      errPAnterior = errP;

      reduccionVelocidad = errP / coeficienteErrorP;// + errI * (1 / coeficienteErrorI) + errD * (1 / coeficienteErrorD);
      
      // errP va entre -2000 y 2000, con p=1/12 reduccionVelocidad va entre -166 y +166 
      // errD va entre -4000 y 4000, con d=1/30 reduccionVelocidad va entre -133 y +133
      // // err_i toma valores entre -32k y 32k, por lo que su aporte a diff_potencia esta acotado entre -32 y +32 (-32 y +32 para 6 sensores)
      // // err_d toma valores entre -5k y 5k, por lo que su aporte a diff_potencia esta acotado entre -inf y +inf (para los niveles de representacion que manejamos). 
      // // Para un caso normal, en que err_p varie 30 entre una medicion y la siguiente, estará acotado entre -45 y +45
      
      // constrain si o no?
      if (reduccionVelocidad < -rangoVelocidad) {
        reduccionVelocidad = -rangoVelocidad;
      } else if (reduccionVelocidad > rangoVelocidad) {
        reduccionVelocidad = rangoVelocidad;
      }
      //reduccionVelocidad = constrain(reduccionVelocidad, -rangoVelocidad, rangoVelocidad);
      
      // printf de valores PID
      //printf("p:%5i i:%5i d:%5i rv:%5i\n", err_p, err_i, err_d, reduccion_velocidad);
      debug("%.5i ", centroDeLinea);
      debug("%.5i ", sensoresLinea);
      debug("%.5i ", errP);
      debug("%.5i\n", reduccionVelocidad);

      if (reduccionVelocidad < 0) {
        // a la derecha de la linea
        analogWrite(pwmMotorI, velocidadMinima + rangoVelocidad - reduccionVelocidad);
        analogWrite(pwmMotorD, velocidadMinima + rangoVelocidad);
      } else {
        // a la izquierda de la linea
        analogWrite(pwmMotorI, velocidadMinima + rangoVelocidad);
        analogWrite(pwmMotorD, velocidadMinima + rangoVelocidad - reduccionVelocidad);
      }
      
      tiempoUs = micros() - tiempoUs;
      
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
