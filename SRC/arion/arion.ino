//Este es el codigo para arion. 

//Definicion de pines del micro.
const int pwmMotorD = 11;
const int pwmMotorI = 10;

const int sentidoMotorD = 3;
const int sentidoMotorI = 5;

const int sensor0 = A0;
const int sensor1 = A1;
const int sensor2 = A2;
const int sensor3 = A3;
const int sensor4 = A4;

const int batteryControl = A5;

const int ledArduino = 13;
const int led1 = 9;
const int led2 = 8;
const int led3 = 12;

const int boton1 = 4;
const int boton2 = 6;
const int boton3 = 7;

//Declaracion de constantes para el codigo.
const int tolerancia = 5; //Margen de ruido al medir negro.
const int MAX_SENSORES = 5;

int sensores[MAX_SENSORES];

boolean estadoActualAdentro;

int ultimoBorde;
const int derecha = 1;
const int izquierda = 0;

// velocidadMinima + rangoVelocidad <= 255 (o explota)
const int velocidadMinima = 0;
const int rangoVelocidad = 255;
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

void setup() {                

  pinMode(pwmMotorD, OUTPUT);
  pinMode(pwmMotorI, OUTPUT);

  pinMode(sentidoMotorD, OUTPUT); 
  pinMode(sentidoMotorI, OUTPUT); 
  
//  pinMode(sensor0, INPUT);
//  pinMode(sensor1, INPUT);
//  pinMode(sensor2, INPUT);
//  pinMode(sensor3, INPUT);
//  pinMode(sensor4, INPUT);

//  pinMode(batteryControl, INPUT);
  pinMode(ledArduino, OUTPUT);
  pinMode(led1, OUTPUT);  
  pinMode(led2, OUTPUT);  
  pinMode(led3, OUTPUT);

  pinMode(boton1, INPUT);
  pinMode(boton2, INPUT);
  pinMode(boton3, INPUT);
  
  estadoActualAdentro = false;
  ultimoBorde = izquierda;
  
  for (int i = 0; i < MAX_SENSORES; i++) {
    sensores[i] = 0;
  }
}

boolean boton1Apretado() {
  return (digitalRead(boton1) == LOW);
}

boolean boton2Apretado() {
  return (digitalRead(boton2) == LOW);
}

boolean boton3Apretado() {
  return (digitalRead(boton3) == LOW);
}

void obtenerSensores() {
  sensores[0] = analogRead(sensor0);
  sensores[1] = analogRead(sensor1);
  sensores[2] = analogRead(sensor2);
  sensores[3] = analogRead(sensor3);
  sensores[4] = analogRead(sensor4);
}

void mostrarSensor(int sensor) {
  if ((sensor >= MAX_SENSORES) || (sensor < 0)) {
    return; 
  }
  digitalWrite(led1, ((sensores[sensor] / 256) ? HIGH : LOW));
  digitalWrite(led2, ((sensores[sensor] / 64) ? HIGH : LOW));
  digitalWrite(led3, ((sensores[sensor] / 16) ? HIGH : LOW));
}

void apagarMotores() {
  analogWrite(pwmMotorD, 0);
  analogWrite(pwmMotorI, 0);
}

void loop() {

  while (!boton2Apretado()) {
    obtenerSensores();
    mostrarSensor(2);
  }
  delay(5); //rebote botón

  while (boton2Apretado());
  delay(5); //rebote botón

  // inicializacion variables pid
  errP = 0;
  errPAnterior = 0;
  errI = 0;
  errD = 0;
  ultimoBorde = izquierda;
  estadoActualAdentro = true;
  
  // arranque progresivo

  while (!boton2Apretado()) {
    obtenerSensores();
    
    if (sensores[0] > 50) {
      ultimoBorde = izquierda;
    } else if (sensores[4] > 50) {
      ultimoBorde = derecha;
    }

    // si me fui, entro en modo "corrección máxima"
    if ((sensores[0] < tolerancia) && (sensores[1] < tolerancia) && (sensores[2] < tolerancia) && (sensores[3] < tolerancia) && (sensores[4] < tolerancia)) {
      estadoActualAdentro = false;
    } else {
      estadoActualAdentro = true;
    } 
    
    if (estadoActualAdentro) {
      // modo pid
      // linea = (0 * s0 + 1000 * s1 + 2000 * s2 + 3000 * s3 + 4000 * s4) / (s0 + s1 + s2 + s3 + s4)
      // 0 a 4000, donde 2000 es el centro
      sensoresLinea = ((int32_t)sensores[1] * 1000 + (int32_t)sensores[2] * 2000 + (int32_t)sensores[3] * 3000 + (int32_t)sensores[4] * 4000) / 
                      ((int32_t)sensores[0] + (int32_t)sensores[1] + (int32_t)sensores[2] + (int32_t)sensores[3] + (int32_t)sensores[4]);
  
      errP = sensoresLinea - centroDeLinea;
      
      errD = errP - errPAnterior;
      //err_i += (err_p >> 8);
      //if ( (err_i >= VALOR_MAX_INT16 - VALOR_MAX_ERR_P) || (err_i <= -(VALOR_MAX_INT16 - VALOR_MAX_ERR_P)) ) {
      //    err_i -= (err_p >> 8);
      //}
      errPAnterior = errP;

      reduccionVelocidad = errP * ( 1 / coeficienteErrorP); //+ err_i * COEFICIENTE_ERROR_I + err_d * COEFICIENTE_ERROR_D;
      
      // printf de valores PID
      //printf("p:%5i i:%5i d:%5i rv:%5i\n", err_p, err_i, err_d, reduccion_velocidad);
  
      // errP va entre -2000 y 2000, con p=1/12 reduccionVelocidad va entre -166 y +166 
      // errD va entre -4000 y 4000, con d=1/30 reduccionVelocidad va entre -133 y +133
      // // err_i toma valores entre -32k y 32k, por lo que su aporte a diff_potencia esta acotado entre -32 y +32 (-32 y +32 para 6 sensores)
      // // err_d toma valores entre -5k y 5k, por lo que su aporte a diff_potencia esta acotado entre -inf y +inf (para los niveles de representacion que manejamos). 
      // // Para un caso normal, en que err_p varie 30 entre una medicion y la siguiente, estará acotado entre -45 y +45
      
      // constrain si o no?
      reduccionVelocidad = constrain(reduccionVelocidad, -rangoVelocidad, rangoVelocidad);
      
      if (reduccionVelocidad < 0) {
        // a la derecha de la linea
        analogWrite(pwmMotorI, velocidadMinima + rangoVelocidad - reduccionVelocidad);
        analogWrite(pwmMotorD, velocidadMinima + rangoVelocidad);
      } else {
        // a la izquierda de la linea
        analogWrite(pwmMotorI, velocidadMinima + rangoVelocidad);
        analogWrite(pwmMotorD, velocidadMinima + rangoVelocidad - reduccionVelocidad);
      }
      
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
  
  // fin de tareas, para poder empezar de nuevo
  apagarMotores();
  delay(5); //rebote botón

  while (boton2Apretado());
  delay(5); //rebote botón

}
