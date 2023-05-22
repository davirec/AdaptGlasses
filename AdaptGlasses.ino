// http://www.esp32learning.com/code/esp32-and-mpu-9250-sensor-example.php
//https://github.com/asukiaaa/MPU9250_asukiaaa
//https://github.com/T-vK/ESP32-BLE-Mouse
//funciona o exemplo do MPU9250_asukiaaa
#include <MPU9250_asukiaaa.h>

#include <BleMouse.h>

BleMouse bleMouse ("AdaptGlasses", "CIN - UFPE", 100);

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
const int sensorIn = 4;
int delayMouse = 10;
int avancoMouse = 20;
int speedMouse = 20;
boolean clickMouse = true;
boolean imprimirVariaveis = false;
unsigned long tempMouse = 0;
boolean initClickMouse = true;
int thresholdMouse = 1000;
int thresholdInfraredClick = 3900;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("started");

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  //
  //  // You can set your own offset for mag values
  //  // mySensor.magXOffset = -50;
  //  // mySensor.magYOffset = -55;
  //  // mySensor.magZOffset = -10;

  bleMouse.begin();
}

void loop() {
  uint8_t sensorId;
  int result;
  unsigned long startTime;

  if (Serial.available() > 0) {
    char comando = Serial.read();

    if (comando == '1') {
      imprimirVariaveis = true;
    } else if (comando == '0') {
      imprimirVariaveis = false;
    }
  }

  result = mySensor.readId(&sensorId);
  if (result == 0) {
    //    Serial.println("sensorId: " + String(sensorId));
  } else {
    Serial.println("Cannot read sensorId " + String(result));
  }

  mySensor.accelUpdate();
  mySensor.gyroUpdate();

  // Obtém os ângulos de inclinação
  float pitch = atan2(mySensor.accelY(), sqrt(pow(mySensor.accelX(), 2) + pow(mySensor.accelZ(), 2))) * 180 / PI;
  float roll = atan2(mySensor.accelX(), sqrt(pow(mySensor.accelY(), 2) + pow(mySensor.accelZ(), 2))) * 180 / PI;
  float yaw = mySensor.gyroZ();
  // Verifica se o giroscópio está apontado para cima, para baixo, esquerda ou direita
  if (pitch > 20) {
    Serial.println("Giroscópio apontado para cima");

    if(avancoMouse < 400){
      avancoMouse += 20;
    }

    if (bleMouse.isConnected()) {
      startTime = millis();
      while (millis() < startTime + avancoMouse) {
        bleMouse.move(0, -1);
        delay(delayMouse);
      }
      //    delay(500);

    }

  } else if (pitch < -20) {
    Serial.println("Giroscópio apontado para baixo");
    if(avancoMouse < 400){
      avancoMouse += 20;
    }
    if (bleMouse.isConnected()) {
      startTime = millis();
      while (millis() < startTime + avancoMouse) {
        bleMouse.move(0, 1);
        delay(delayMouse);
      }
      //    delay(500);

    }
  } else if (roll > 20) {
    Serial.println("Giroscópio apontado para a direita");
    if(avancoMouse < 400){
      avancoMouse += 20;
    }
    if (bleMouse.isConnected()) {
      startTime = millis();
      while (millis() < startTime + avancoMouse) {
        bleMouse.move(1, 0);
        delay(delayMouse);
      }
    }

  } else if (roll < -20) {
    Serial.println("Giroscópio apontado para a esquerda");

    if(avancoMouse < 400){
      avancoMouse += 20;
    }

    if (bleMouse.isConnected()) {
      startTime = millis();
      while (millis() < startTime + avancoMouse) {
        bleMouse.move(-1, 0);
        delay(delayMouse);
      }
    }
  } else{
    avancoMouse = speedMouse;
  }

  if (bleMouse.isConnected() and analogRead(sensorIn) < thresholdInfraredClick) {
//    Serial.println("1");
    if(initClickMouse){
      tempMouse = millis();
      initClickMouse = false;
//      Serial.println("2");
    }

    if (millis() - tempMouse >= thresholdMouse) {
      bleMouse.click(MOUSE_LEFT);
      initClickMouse = true;
      tempMouse = millis();
      Serial.println("********* Click Mouse *********");
    }

//    if (clickMouse) {
//      bleMouse.click(MOUSE_LEFT);
//      clickMouse = false;
//      Serial.println("********* Click Mouse *********");
//    }
    //    bleMouse.click(MOUSE_LEFT);
    //    bleMouse.click(MOUSE_RIGHT);

    //    delay(100);
  } else {
//    clickMouse = true;
    tempMouse = millis();
    initClickMouse = true;
//    Serial.println("3");
  }

  // Verifica se há rotação no eixo Z (yaw)
  if (yaw > 45) {
    Serial.println("Rotação no sentido horário (Z)");

  } else if (yaw < -45) {
    Serial.println("Rotação no sentido anti-horário (Z)");

  }

  if (imprimirVariaveis) {
    Serial.println("sensorIn " + String(analogRead(sensorIn)) + " pitch " + String(pitch) + " roll " + String(roll) + " yaw " + yaw);
  }
  //delay(100);
  Serial.println(avancoMouse);



  //  result = mySensor.accelUpdate();
  //  if (result == 0) {
  //    aX = mySensor.accelX();
  //    aY = mySensor.accelY();
  //    aZ = mySensor.accelZ();
  //    aSqrt = mySensor.accelSqrt();
  //    Serial.println("accelX: " + String(aX));
  //    Serial.println("accelY: " + String(aY));
  //    Serial.println("accelZ: " + String(aZ));
  //    Serial.println("accelSqrt: " + String(aSqrt));
  //  } else {
  //    Serial.println("Cannod read accel values " + String(result));
  //  }

  //  result = mySensor.gyroUpdate();
  //  if (result == 0) {
  //    gX = mySensor.gyroX();
  //    gY = mySensor.gyroY();
  //    gZ = mySensor.gyroZ();
  //    Serial.println("gyroX: " + String(gX));
  //    Serial.println("gyroY: " + String(gY));
  //    Serial.println("gyroZ: " + String(gZ));
  //  } else {
  //    Serial.println("Cannot read gyro values " + String(result));
  //  }
  //
  //  result = mySensor.magUpdate();
  //  if (result != 0) {
  //    Serial.println("cannot read mag so call begin again");
  //    mySensor.beginMag();
  //    result = mySensor.magUpdate();
  //  }
  //  if (result == 0) {
  //    mX = mySensor.magX();
  //    mY = mySensor.magY();
  //    mZ = mySensor.magZ();
  //    mDirection = mySensor.magHorizDirection();
  //    Serial.println("magX: " + String(mX));
  //    Serial.println("maxY: " + String(mY));
  //    Serial.println("magZ: " + String(mZ));
  //    Serial.println("horizontal direction: " + String(mDirection));
  //  } else {
  //    Serial.println("Cannot read mag values " + String(result));
  //  }

  //  Serial.println("at " + String(millis()) + "ms");
  //  Serial.println(""); // Add an empty line
  //  delay(500);
}
