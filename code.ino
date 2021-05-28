#include <AFMotor.h>
#include <I2Cdev.h>
#include <PID_v2.h>
#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

#define PIN_CHANNEL1_A 28
#define PIN_CHANNEL2_A 30
#define PIN_CHANNEL3_A 32
#define PIN_CHANNEL4_A 34

#define PIN_CHANNEL1_B 29
#define PIN_CHANNEL2_B 31
#define PIN_CHANNEL3_B 33
#define PIN_CHANNEL4_B 35

#define MAX_POWER_RADIANS 0.92

#define PULSES_PER_ROTATION 1497
#define ENCODER_SAMPLES 100
#define GYRO_ROTATION_QUARTER 350859
#define ORIENTATION_SAMPLES 100
#define SPEED_CALC_PERIOD 100

#define BATT_PIN A10

/*
   Nuestro sensor de aceleración nos muestra la aceleración en los ejes X, Y, Z
   y la aceleración angular X, Y Z; es decir, es un sensor de 6 ejes. Dependiendo de
   la posición en la que se encuentra el sensor, los valores de aceleración angular
   tienen un valor constante de offset. Lo primero que hacemos antes de realizar cualquier
   movimiento es calibrar el sensor para hacer una media del ruido y establecer los
   offsets del sensor. Únicamente medimos el offset del eje Z, ya que es el que más nos
   importa para llevar un control sobre la orientación.

   https://upload.wikimedia.org/wikipedia/commons/c/c1/Yaw_Axis_Corrected.svg (Yaw)
*/

const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);
long lastAngZCalc = 0;
int ax, ay, az, gx, gy, gz;
int angZAxisCalibration[ORIENTATION_SAMPLES] = {0};
int angZAxisSamples[ORIENTATION_SAMPLES] = {0};
int angZNoiseBound[2] = { -32768, 32767};
int angZOffset = 0;
int calibrationIndex = 0;
int gyroSampleIndex = 0;
bool gyroCalibrated = false;
int gyroCalibrationIndex = 0;
float angZ = 0;
float targetAngZ = 0;
double swervePower = 0;

/*
   Los motores tienen dos velocidades distintas; la velocidad objetiva y la velocidad
   corregida. Dependiendo de la orientación del vehículo, los motores de un lado o de
   otro tienen más potencia. Este multiplicador de potencia afecta a la velocidad
   corregida.

   El control PID de la velocidad de los motores se realiza a partir de la velocidad
   corregida, no de la velocidad objetiva. La velocidad corregida y la velocidad
   objetiva son coincidentes si el vehículo está apuntando exactamente a su orientación
   objetiva.

   La velocidad corregida es calculada a través del output del PID del sensor de
   orientación.

   Para más información en cómo modular la velocidad de los motores directamente,
   puedes leer la librería del puente en H que se encarga de la frecuéncia de
   funcionamiento y de la orientación a nivel de alimentación de los motores:
   https://cdn-learn.adafruit.com/downloads/pdf/adafruit-motor-shield.pdf
*/

AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR12_64KHZ);
AF_DCMotor motor4(4, MOTOR12_64KHZ);

double targetSpeedMotor1 = 0;
double targetSpeedMotor2 = 0;
double targetSpeedMotor3 = 0;
double targetSpeedMotor4 = 0;

double correctedTargetSpeedMotor1 = 0;
double correctedTargetSpeedMotor2 = 0;
double correctedTargetSpeedMotor3 = 0;
double correctedTargetSpeedMotor4 = 0;

double correctedSpeedMotor1 = 0;
double correctedSpeedMotor2 = 0;
double correctedSpeedMotor3 = 0;
double correctedSpeedMotor4 = 0;

/*
   Bateria
*/
float battVoltage = 0;

/*
   Para poder determinar la posición, velocidad y aceleración de nuestros motores,
   realizamos un registro de los flancos positivos de un solo pin de nuestro encoder.
   Cabe destacar que de esta manera *NO* estamos perdiendo resolución, ya que el
   segundo pin es básicamente el primer pin con un desfase de 90 grados.

   Guardando el timestamp de cada cambio podemos ver la diferencia de tiempos entre
   cambios y de esta manera saber la frecuencia media. Solo realizamos la operación
   de cálculo de posición, velocidad y aceleración cada cierto tiempo.

   Para poder determinar el sentido de giro de nuestro encoder, solo debemos comprobar
   la señal de nuestro segundo pin en el momento de un flanco positivo del primer pin,
   o viceversa. Cuando giramos en el sentido horario, el segundo pin estara en off en
   el flanco positivo, y en sentido antihorario, el segundo pin estará en on.

   Para entender mejor este concepto, recomendamos ver esta página web donde se
   muestra un ejemplo interactivo bajo el apartado "Understanding Quadrature Encoded Signals"
   https://www.pjrc.com/teensy/td_libs_Encoder.html
*/

bool encoderMotor1LastValue = false;
bool encoderMotor2LastValue = false;
bool encoderMotor3LastValue = false;
bool encoderMotor4LastValue = false;

int encoderMotor1Pointer = 0;
int encoderMotor2Pointer = 0;
int encoderMotor3Pointer = 0;
int encoderMotor4Pointer = 0;

long encoderMotor1[ENCODER_SAMPLES] = {0};
long encoderMotor2[ENCODER_SAMPLES] = {0};
long encoderMotor3[ENCODER_SAMPLES] = {0};
long encoderMotor4[ENCODER_SAMPLES] = {0};

/* Valores computados en el momento del flanco: */

bool clockwiseMotor1 = false;
bool clockwiseMotor2 = false;
bool clockwiseMotor3 = false;
bool clockwiseMotor4 = false;

/* Valores computados cada cierto tiempo: */

long lastSpeedCalc = 0;

double speedMotor1 = 0;
double speedMotor2 = 0;
double speedMotor3 = 0;
double speedMotor4 = 0;

double comulativeSpeedMotor1 = 0;
double comulativeSpeedMotor2 = 0;
double comulativeSpeedMotor3 = 0;
double comulativeSpeedMotor4 = 0;

PID_v2 PID1(1, 1.4, 0.025, PID::Direct);
PID_v2 PID2(1, 1.4, 0.025, PID::Direct);
PID_v2 PID3(1, 1.4, 0.025, PID::Direct);
PID_v2 PID4(1, 1.4, 0.025, PID::Direct);

void setup() {

  /*
     Nuestro sensor de orientación y distáncia utiliza la comunicación I2C para
     enviar los datos de aceleración a nuestro Arduino. El bus de comunicación I2C
     es extremadamente sensible al ruido electromagnético.

     Es posible que merezca la pena aislar mejor el bus de comunicación del
     sensor en un futuro para evitar que el Arduino se quede pillado en el timeout,
     perdiendose las lecturas de los encoders, mensajes por los puertos serie
     y otros sensores.

     El primer parámetro de esta función indica nuestro tiempo de timeout y el
     segundo parámetro indica si el sincronismo debe ser restaturado al bus para
     intentar restablecer la comunicación con los dispositivos.

     Esta función no tiene porque funcionar en todos los modelos de Arduino ya que
     depende de la librería Wire, propia del firmware de cada Arduino.

     https://github.com/arduino/Arduino/issues/10803
     https://github.com/jrowberg/i2cdevlib/issues/252
  */
  mpu.initialize();
  delay(500);
  Wire.setWireTimeout(200, true);

  Serial.begin(1000000);

  pinMode(BATT_PIN, INPUT);

  PID1.Start(0, 0, 0);
  PID2.Start(0, 0, 0);
  PID3.Start(0, 0, 0);
  PID4.Start(0, 0, 0);

}

void loop() {

  /*
     El giroscopio se lee en cada iteración. La función calcula automáticamente
     la orientación si el buffer de samples del giroscopio se ha llenado.

     En el caso de que el buffer se haya llenado, se computa el control de
     actuación para hacer un lazo cerrado utilizando consignas de velocidad
     individualizadas.
  */
  readBatt();

  if (performGyroRead()) {
    // Update Gyro Actuation Output

    swervePower = (double) angZ / (double) GYRO_ROTATION_QUARTER;
    if (swervePower > 1) {
      swervePower = 1;
    } else if (swervePower < -1) {
      swervePower = -1;
    }
  }

  /*
     Se leen todos los encoders en cada iteración. Esto no quiere decir que se
     calcule la velocidad en cada iteración. El cálculo de velocidad solo se
     realiza cada 100ms.
  */
  if (gyroCalibrated) {
    readAllEncoders();
    if (timeControl(lastSpeedCalc, 100)) {
      updateAllSpeeds();


      float powerLeft = swervePower + 1;
      float powerRight = -swervePower + 1;

      correctedTargetSpeedMotor1 = targetSpeedMotor1 * powerRight;
      correctedTargetSpeedMotor2 = targetSpeedMotor2 * powerLeft;
      correctedTargetSpeedMotor3 = targetSpeedMotor3 * powerRight;
      correctedTargetSpeedMotor4 = targetSpeedMotor4 * powerLeft;

      PID1.Setpoint(correctedTargetSpeedMotor1);
      PID2.Setpoint(correctedTargetSpeedMotor2);
      PID3.Setpoint(correctedTargetSpeedMotor3);
      PID4.Setpoint(correctedTargetSpeedMotor4);

      correctedSpeedMotor1 = PID1.Run(speedMotor1);
      correctedSpeedMotor2 = PID2.Run(speedMotor2);
      correctedSpeedMotor3 = PID3.Run(speedMotor3);
      correctedSpeedMotor4 = PID4.Run(speedMotor4);

      motor1.run(correctedSpeedMotor1 >= 0 ? FORWARD : BACKWARD);
      motor2.run(correctedSpeedMotor2 >= 0 ? FORWARD : BACKWARD);
      motor3.run(correctedSpeedMotor3 >= 0 ? FORWARD : BACKWARD);
      motor4.run(correctedSpeedMotor4 >= 0 ? FORWARD : BACKWARD);

      motor1.setSpeed(radiansToPower(correctedSpeedMotor1));
      motor2.setSpeed(radiansToPower(correctedSpeedMotor2));
      motor3.setSpeed(radiansToPower(correctedSpeedMotor3));
      motor4.setSpeed(radiansToPower(correctedSpeedMotor4));

      String speedString = "{'c':[" + String(speedMotor1) + "," + String(speedMotor2) + "," + String(speedMotor3) + "," + String(speedMotor4) + "],'t':[" + String(targetSpeedMotor1) + "," + String(targetSpeedMotor2) + "," + String(targetSpeedMotor3) + "," + String(targetSpeedMotor4) + "],'p':[" + String(correctedSpeedMotor1) + "," + String(correctedSpeedMotor2) + "," + String(correctedSpeedMotor3) + "," + String(correctedSpeedMotor4) + "],'n':[" + String(correctedTargetSpeedMotor1) + "," + String(correctedTargetSpeedMotor2) + "," + String(correctedTargetSpeedMotor3) + "," + String(correctedTargetSpeedMotor4) + "]}";
      String orientationString = "{'o':" + String(angZ) + ",'q':" + String(GYRO_ROTATION_QUARTER) + ",'o':" + String(angZOffset) + ",'t':" + String(targetAngZ) + ",'n':[" + String(angZNoiseBound[0]) + "," + String(angZNoiseBound[1]) + "]}";
      String battString = "{'v':" + String(battVoltage) + "}";
      Serial.println("{'s':" + speedString + ",'b':" + battString + ",'o':" + battString + "}");

    }
  }


 
  if (Serial.available()) {
    String incomingString = Serial.readStringUntil('\n');
    String stringWithutType = incomingString.substring(2);
    String type = incomingString.substring(0, 2);
    if (type == "t:") {
      // new targets
      targetSpeedMotor1 = (float) getValue(stringWithutType, ':', 0).toInt() / (float) 100;
      targetSpeedMotor2 = (float) getValue(stringWithutType, ':', 1).toInt() / (float) 100;
      targetSpeedMotor3 = (float) getValue(stringWithutType, ':', 2).toInt() / (float) 100;
      targetSpeedMotor4 = (float) getValue(stringWithutType, ':', 3).toInt() / (float) 100;
    } else if (type == "o:") {
      targetAngZ = stringWithutType.toInt();
    }
  }


}


void readBatt() {
  battVoltage = ((float) analogRead(BATT_PIN) / 1024) * 9;
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

int radiansToPower(double radiansv) {
  // 0.92 = 255
  radiansv = abs(radiansv);
  if (radiansv > MAX_POWER_RADIANS) {
    return 255;
  } else {
    return ((double)radiansv / (double) MAX_POWER_RADIANS) * 255;
  }
}

/*
   ----------------------------
   | TIMING CONTROL            |
   ---------------------------
*/

bool timeControl(long &lastTime, int period) {
  if (millis() - lastTime > period) {
    lastTime = millis();
    return true;
  } else {
    return false;
  }
}

/*
   ----------------------------
   | SPEED CONTROL            |
   ----------------------------
*/

void readAllEncoders() {
  readEncoder(digitalRead(PIN_CHANNEL1_A), digitalRead(PIN_CHANNEL1_B), encoderMotor1Pointer, clockwiseMotor1, encoderMotor1LastValue, encoderMotor1);
  readEncoder(digitalRead(PIN_CHANNEL2_A), digitalRead(PIN_CHANNEL2_B), encoderMotor2Pointer, clockwiseMotor2, encoderMotor2LastValue, encoderMotor2);
  readEncoder(digitalRead(PIN_CHANNEL3_A), digitalRead(PIN_CHANNEL3_B), encoderMotor3Pointer, clockwiseMotor3, encoderMotor3LastValue, encoderMotor3);
  readEncoder(digitalRead(PIN_CHANNEL4_A), digitalRead(PIN_CHANNEL4_B), encoderMotor4Pointer, clockwiseMotor4, encoderMotor4LastValue, encoderMotor4);
}

void readEncoder(int channelA, int channelB, int &pointer, bool &orientation, bool &lastValue, long *buffer) {
  if (channelA != lastValue) {
    if (!lastValue) {
      // rising edge
      // orientation
      orientation = channelB;
      // timestamp
      buffer[pointer] = micros();

      pointer++;
      if (pointer >= ENCODER_SAMPLES) pointer = 0;
    }
    // positive or negative edge
    lastValue = channelA;
  }
}

int countFromCyclicArray(long *targetArray, int index, long until) {
  index += -1;
  if (index < 0) {
    index = ENCODER_SAMPLES - 1;
  }
  int untilIndex = index - ENCODER_SAMPLES;
  long val = 0;
  int count = 0;
  for (index + ENCODER_SAMPLES; index > untilIndex; index += -1) {
    if (index < 0) {
      val = targetArray[ENCODER_SAMPLES + index];
    } else {
      val = targetArray[index];
    }
    if (val <= 0 || val < until) {
      return count;
    } else {
      count++;
    }
  }
  return count;
}

float pulsesToRadians(int pulses, int totalPulses, long microsv) {
  if (pulses > 0) {
    float radiansPerDefinedTime = ((float) pulses * (float) 2 * (float) 3.14) / (float) totalPulses;
    return ((float) radiansPerDefinedTime * (float) (SPEED_CALC_PERIOD) / (float) microsv) * 10000;
  } else {
    return 0;
  }
}

void updateAllSpeeds() {

  speedMotor1 = getEncoderSpeed(encoderMotor1, encoderMotor1Pointer) * (clockwiseMotor1 ? 1 : -1);
  speedMotor2 = getEncoderSpeed(encoderMotor2, encoderMotor2Pointer) * (clockwiseMotor2 ? -1 : 1);
  speedMotor3 = getEncoderSpeed(encoderMotor3, encoderMotor3Pointer) * (clockwiseMotor3 ? 1 : -1);
  speedMotor4 = getEncoderSpeed(encoderMotor4, encoderMotor4Pointer) * (clockwiseMotor4 ? -1 : 1);

}


float getEncoderSpeed(long *buffer, int bufferIndex) {
  return pulsesToRadians(countFromCyclicArray(buffer, bufferIndex, micros() - ((long) SPEED_CALC_PERIOD * (long) 1000)), PULSES_PER_ROTATION, (long) SPEED_CALC_PERIOD * (long) 1000);
}

/*
   ----------------------------
   | MPU 6050                 |
   ----------------------------
*/

bool performGyroRead() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if (!gyroCalibrated) {
    gyroCalibrated = calibrateGyro(gz, angZAxisCalibration, angZNoiseBound, angZOffset, calibrationIndex);
  } else if (addToGyroBuffer(angZAxisSamples, gyroSampleIndex, gz)) {
    getNewOrientation(angZ, getGyroBufferSumAndEmpty(angZAxisSamples), lastAngZCalc);
    return true;
  } else {
    return false;
  }
}

float getNewOrientation(float &orientation, long sum, long &timeControl) {
  orientation += sum * ((millis() - (float) timeControl) / 1000);
  timeControl = millis();
  if (abs(orientation) > GYRO_ROTATION_QUARTER * 4) orientation = 0;
  return orientation;
}

long getGyroBufferSumAndEmpty(int *buffer) {
  long sum = 0;
  for (int i = 0; i < ORIENTATION_SAMPLES; i++) {
    sum += buffer[i];
    buffer[i] = 0;
  }
  return sum;
}

bool addToGyroBuffer(int *buffer, int &index, int val) {
  if (!isGyroNoise(val)) {
    buffer[index] = val + angZOffset;
  } else {
    buffer[index] = 0;
  }
  index++;
  if (index > ORIENTATION_SAMPLES) {
    index = 0;
    return true;
  } else {
    return false;
  }
}

bool isGyroNoise(int val) {
  return (val < angZNoiseBound[0] && val > angZNoiseBound[1]);
}

bool calibrateGyro(int value, int *dataArray, int *noiseData, int &offset, int &index) {
  if (value != 0) {
    dataArray[index] = value;
    if (value > noiseData[0]) {
      noiseData[0] = value + abs(value) * 0.5;
    } else if (value < noiseData[1]) {
      noiseData[1] = value - abs(value) * 0.5;
    }
    index++;
    if (index > ORIENTATION_SAMPLES - 1) {
      int long total = 0;
      for (int i = 0; i < ORIENTATION_SAMPLES; i++) {
        total += dataArray[i];
      }
      offset = total / ORIENTATION_SAMPLES;
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}
