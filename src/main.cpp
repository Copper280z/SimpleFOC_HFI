#include <Arduino.h>
#include "pinout.h"
#include "SimpleFOC.h"
#include "HFIBLDCMotor.h"

HFIBLDCMotor motor = HFIBLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOT1_A, MOT1_B, MOT1_C, MOT1_EN);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.01f, CS_A, CS_B);

void setup() {
  Serial.begin(250000);

  driver.voltage_power_supply = 8;
  driver.init();

  motor.linkDriver(&driver);
}

void loop() {
}
