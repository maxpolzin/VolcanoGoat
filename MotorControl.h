#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
  const int SERVO_MIN = 500;
  const int SERVO_MAX = 2500;
  const int PWM_TIMER_12_BIT = 12;
  const int PWM_BASE_FREQ = 333;
  const int PWM_LEFT_CHANNEL;
  const int PWM_RIGHT_CHANNEL;

  const int PWM_CAMERA_BASE_FREQ = 50;
  const int CAMERA_SERVO_MIN = 700;
  const int CAMERA_SERVO_MAX = 2200;
  const int PWM_CAMERA_CHANNEL;

public:
  MotorControl(int leftChannel, int rightChannel, int cameraChannel);
  void begin(int leftPin, int rightPin, int cameraPin);
  void update(double forwardVelocityCommand, double steeringVelocityCommand, double cameraPositionCommand);

private:
  void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255);
};

#endif
