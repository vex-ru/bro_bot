# bro_bot
Прототип робота для уборки мелкого мусора 


# Оборудование

- Тележка с 2-мя моторами
    - VexV5 Arduino Shield (Mega2560 <--> Serial2 <--> ESP32)
    - VexV5 Motor With Encoder
    - Металлический каркас - заднеприводный (VEX5_PORT_1 - правое колесо, VEX5_PORT_5 - левое)

- Манипулятор уловой с 6 моторами Dynamixel.
- TrackengCam v3 - https://wiki.appliedrobotics.ru/metod/Trackingcam_v3/
- Ноутбук с Ubuntu 20.04
- Wifi с именем POSIL_LAN

# Примеры кода

- Движение вперед мотором VexV5 https://github.com/vex-ru/VexV5

```cpp
#include <Vex5.h>

#define BAUDRATE         (500000) //SPI speed
#define SPI_NSS_PIN      (-1)     //Use jumper
#define IS_RESET_SHIELD  (0)      //Keep servo positions during reset

Vex5_Motor motor;
int32_t goalPosition = 0;
int16_t maxSpeed = 200;
int16_t maxCurrent_mA = 0; //0 - unlimited
int16_t realSpeed = 0;
int32_t realPosition = 0;
int16_t realPower = 0;

unsigned long time = millis();

void setup() {
  Serial.begin(115200);
  Vex5.begin(BAUDRATE, SPI_NSS_PIN, IS_RESET_SHIELD);
  motor.begin(VEX5_PORT_1);
  motor.resetEncoder();
  time = millis();
}

void loop() {
  if(millis() - time > 1000 && realSpeed == 0) {
    time = millis();
    goalPosition = goalPosition + Vex5.deg_to_ticks(90);
    motor.setMaxCurrent(maxCurrent_mA);
    motor.setPosition(goalPosition, maxSpeed);
  } 
  motor.getSpeed(realSpeed);
  motor.getPosition(realPosition);
  delay(10);
  motor.getPower(realPower);
  Serial.print("Goal Position:\t" + String(goalPosition) + "\t");
  Serial.println("Real Position Speed Power:\t" + String(realPosition) + "\t" + String(realSpeed) + "\t" + String(realPower));
  delay(300);
}
```

- Движение Dynamixel мотором библиотекой https://github.com/vex-ru/dxlmaster2

```cpp
// Test motor joint mode


#include "DxlMaster2.h"

// id of the motor
const uint8_t id = 1;
// speed, between 0 and 1023
int16_t speed = 512;
// communication baudrate
const long unsigned int baudrate = 57600;

DynamixelMotor motor(id);

void setup()
{ 
  DxlMaster.begin(baudrate);
  motor.protocolVersion(2);
  delay(100);
  
  // check if we can communicate with the motor
  // if not, we stop here
  uint8_t status = motor.init();
  if(status != DYN_STATUS_OK)
  {
    while(1);
  }

  motor.enableTorque();

  // set to joint mode, with a 180° angle range
  // see robotis doc to compute angle values
  motor.jointMode(204, 820);
  motor.speed(speed);
}

void loop() 
{
  // go to middle position
  motor.goalPosition(512);
  delay(500);

  // move 45° CCW
  motor.goalPosition(666);
  delay(500);

  // go to middle position
  motor.goalPosition(512);
  delay(500);

  // move 45° CW
  motor.goalPosition(358);
  delay(500);
}

```

- ROS пакет для работы с TrackingCam v3 https://github.com/AppliedRobotics/tc3-ros-package

Инструкция - https://wiki.appliedrobotics.ru/metod/Trackingcam_v3/g_5.html#connecting-trackingcam-v3-module-to-ros-operating-system