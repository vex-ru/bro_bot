/**
 * bro_bot_mega2560.ino
 * Низкоуровневый драйвер шасси на VexV5 для дифференциального привода
 * 
 * Аппаратная конфигурация:
 * - VEX5_PORT_1 : правое колесо
 * - VEX5_PORT_5 : левое колесо
 * - Serial2 (115200) : связь с ESP32
 * 
 * Протокол команд (текстовый, \n-терминированный):
 * - "S:<l>,<r>\n"   : установить скорость левого/правого мотора (-1000..1000)
 * - "P\n"           : запрос позиции → "POS:<x_mm>,<y_mm>,<heading_deg>\n"
 * - "H\n"           : запрос курса → "HEAD:<deg>\n"
 * - "E\n"           : сырые значения энкодеров → "ENC:<left>,<right>\n"
 * - "R\n"           : сброс одометрии и энкодеров
 * - "C:<tpm>,<wb>\n": калибровка: ticks_per_meter, wheel_base_mm
 * - "V\n"           : версия прошивки
 */
#include <Vex5.h>

// ========== КОНФИГУРАЦИЯ ==========
// 🔧 ДЛЯ ТЕСТА ЧЕРЕЗ USB: раскомментируйте строку ниже
#define CMD_SERIAL        Serial
// 🔌 ДЛЯ РАБОТЫ С ESP32: раскомментируйте эту строку вместо верхней
// #define CMD_SERIAL      Serial2

#define CMD_BAUD          115200
#define DEBUG_SERIAL      Serial
#define DEBUG_BAUD        115200
#define CMD_BUFFER_SIZE   64
#define SERIAL_BUF_SIZE   128

// Кинематика робота (калибруется экспериментально!)
#define WHEEL_RADIUS_MM   50.0f
#define TICKS_PER_REV     1440.0f
#define WHEEL_BASE_MM     250.0f

#define TICKS_PER_METER   (TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_MM / 1000.0f))
#define MM_PER_TICK       (1000.0f / TICKS_PER_METER)

#define USE_PID           false
#define KP_SPEED          0.8f
#define KI_SPEED          0.02f
#define KD_SPEED          0.1f

// Буферы для форматирования строк (вместо printf)
#define SERIAL_BUF_SIZE   128
char cmdResponse[SERIAL_BUF_SIZE];
char debugBuf[SERIAL_BUF_SIZE];

// ========== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========
Vex5_Motor motorLeft, motorRight;

struct Odometry {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float heading_deg = 0.0f;
  int32_t left_ticks = 0;
  int32_t right_ticks = 0;
  unsigned long last_update = 0;
} odom;

struct PID {
  float kp, ki, kd;
  float integral = 0, last_error = 0;
  unsigned long last_time = 0;
  
  // Конструктор для инициализации
  PID() : kp(0), ki(0), kd(0) {}
  PID(float p, float i, float d) : kp(p), ki(i), kd(d) {}
  
  void begin(float p, float i, float d) {
    kp = p; ki = i; kd = d;
    reset();
  }
  
  int16_t compute(int16_t target, int16_t current) {
    if (!USE_PID) return target;
    
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0f;
    if (dt < 0.001f) return current;
    
    float error = target - current;
    integral += error * dt;
    integral = constrain(integral, -500, 500);
    float derivative = (error - last_error) / dt;
    
    last_error = error;
    last_time = now;
    
    int16_t output = kp * error + ki * integral + kd * derivative;
    return constrain(output, -1000, 1000);
  }
  
  void reset() { integral = 0; last_error = 0; last_time = millis(); }
} pidLeft, pidRight;

char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIdx = 0;
bool cmdReady = false;

// ========== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ==========
inline float ticksToMm(int32_t ticks) {
  return ticks * MM_PER_TICK;
}

inline float normalizeHeading(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

// Безопасная отправка форматированной строки в Serial
void sendCmd(const char* format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(cmdResponse, SERIAL_BUF_SIZE, format, args);
  va_end(args);
  CMD_SERIAL.print(cmdResponse);
}

void sendDebug(const char* format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(debugBuf, SERIAL_BUF_SIZE, format, args);
  va_end(args);
  DEBUG_SERIAL.print(debugBuf);
}

// ========== ОДОМЕТРИЯ ==========
void updateOdometry() {
  unsigned long now = millis();
  float dt = (now - odom.last_update) / 1000.0f;
  if (dt < 0.01f) return;
  
  odom.last_update = now;
  
  int32_t left_now, right_now;
  motorLeft.getPosition(left_now);
  motorRight.getPosition(right_now);
  
  int32_t dLeft = left_now - odom.left_ticks;
  int32_t dRight = right_now - odom.right_ticks;
  odom.left_ticks = left_now;
  odom.right_ticks = right_now;
  
  float distLeft = ticksToMm(dLeft);
  float distRight = ticksToMm(dRight);
  
  float distCenter = (distLeft + distRight) / 2.0f;
  float dHeading = (distRight - distLeft) / WHEEL_BASE_MM;
  
  float heading_rad = odom.heading_deg * M_PI / 180.0f;
  odom.x_mm += distCenter * cos(heading_rad);
  odom.y_mm += distCenter * sin(heading_rad);
  odom.heading_deg = normalizeHeading(odom.heading_deg + dHeading * 180.0f / M_PI);
}

// ========== УПРАВЛЕНИЕ МОТОРАМИ ==========
void setSpeed(int16_t left, int16_t right) {
  left = constrain(left, -1000, 1000);
  right = constrain(right, -1000, 1000);
  
  if (USE_PID) {
    int16_t realLeft, realRight;
    motorLeft.getSpeed(realLeft);
    motorRight.getSpeed(realRight);
    left = pidLeft.compute(left, realLeft);
    right = pidRight.compute(right, realRight);
  }
  
  motorLeft.setSpeed(left);
  motorRight.setSpeed(-right);  // инверсия из-за механики
}

void resetOdometry() {
  motorLeft.resetEncoder();
  motorRight.resetEncoder();
  odom = Odometry();
  pidLeft.reset();
  pidRight.reset();
}

void setCalibration(float tpm, float wb_mm) {
  sendDebug("CAL: tpm=%.2f wb=%.1f\n", tpm, wb_mm);
}

// ========== КОМАНДНЫЙ ИНТЕРФЕЙС ==========
void parseCommand(char* cmd) {
  char* end = cmd + strlen(cmd) - 1;
  while (end > cmd && (*end == '\n' || *end == '\r')) *end-- = '\0';
  
  if (strlen(cmd) < 1) return;
  
  switch (cmd[0]) {
    case 'S': {
      if (cmd[1] != ':') break;
      int16_t l, r;
      if (sscanf(cmd + 2, "%hd,%hd", &l, &r) == 2) {
        setSpeed(l, r);
        CMD_SERIAL.println("OK");
      } else {
        CMD_SERIAL.println("ERR:SYNTAX");
      }
      break;
    }
    case 'P': {
      updateOdometry();
      sendCmd("POS:%.1f,%.1f,%.2f\n", odom.x_mm, odom.y_mm, odom.heading_deg);
      break;
    }
    case 'H': {
      updateOdometry();
      sendCmd("HEAD:%.2f\n", odom.heading_deg);
      break;
    }
    case 'E': {
      int32_t l, r;
      motorLeft.getPosition(l);
      motorRight.getPosition(r);
      sendCmd("ENC:%ld,%ld\n", l, r);
      break;
    }
    case 'R': {
      resetOdometry();
      CMD_SERIAL.println("OK");
      break;
    }
    case 'C': {
      if (cmd[1] != ':') break;
      float tpm, wb;
      if (sscanf(cmd + 2, "%f,%f", &tpm, &wb) == 2) {
        setCalibration(tpm, wb);
        CMD_SERIAL.println("OK");
      } else {
        CMD_SERIAL.println("ERR:SYNTAX");
      }
      break;
    }
    case 'V': {
      CMD_SERIAL.println("VER:bro_bot_mega2560/1.1");
      break;
    }
    default:
      CMD_SERIAL.println("ERR:UNKNOWN");
      break;
  }
}

void checkCommand() {
  while (CMD_SERIAL.available() && !cmdReady) {
    char c = CMD_SERIAL.read();
    if (c == '\n' || cmdIdx >= CMD_BUFFER_SIZE - 1) {
      cmdBuffer[cmdIdx] = '\0';
      cmdReady = true;
      cmdIdx = 0;
    } else if (c != '\r') {
      cmdBuffer[cmdIdx++] = c;
    }
  }
  
  if (cmdReady) {
    parseCommand(cmdBuffer);
    cmdReady = false;
  }
}

// ========== SETUP / LOOP ==========
void setup() {
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  while (!DEBUG_SERIAL);
  sendDebug("[INIT] bro_bot_mega2560 ready\n");
  
  CMD_SERIAL.begin(CMD_BAUD);
  
  Vex5.begin();
  motorLeft.begin(VEX5_PORT_5);
  motorRight.begin(VEX5_PORT_1);
  
  motorLeft.resetEncoder();
  motorRight.resetEncoder();
  
  // Инициализация PID через метод begin()
  pidLeft.begin(KP_SPEED, KI_SPEED, KD_SPEED);
  pidRight.begin(KP_SPEED, KI_SPEED, KD_SPEED);
  
  delay(500);
  CMD_SERIAL.println("OK");
}

void loop() {
  checkCommand();
  
  static unsigned long odomTimer = 0;
  if (millis() - odomTimer >= 20) {
    updateOdometry();
    odomTimer = millis();
  }
  
  static unsigned long debugTimer = 0;
  if (millis() - debugTimer >= 500) {
    sendDebug("[ODOM] x=%.1fmm y=%.1fmm h=%.1f° | ENC L=%ld R=%ld\n",
      odom.x_mm, odom.y_mm, odom.heading_deg,
      odom.left_ticks, odom.right_ticks);
    debugTimer = millis();
  }
  
  delay(2);
}
