#pragma once
#include <util/atomic.h>
#include <Arduino.h>


class MotorController {

  public:  
    MotorController(int pwm, int in1, int in2);
    ~MotorController() = default;
    void setMotor();
    void readEncoder(volatile unsigned currentTimeTick);
    void pidController(float setPoint, float deltaT);
    void setPID(const float& kp, const float& ki, const float& kd);
    float getMotorSpeed();
  private:
    //Instances
    int posPrev{0};
    volatile int pos_i{0};
    float v1Filt{0};
    float v1Prev{0};
    float eintegral{0};
    float eprev{0.0};
    int pwm_{0};
    int in1_{0};
    int in2_{0};
    float dir_{0.0};
    int pwm_val_{0};
    float kp_{0.0};
    float ki_{0.0};
    float kd_{0.0};
    volatile unsigned previousTimeTick{0};
};
