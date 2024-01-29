#include <Arduino.h>
#include "pinout.h"
#include "SimpleFOC.h"


HFIBLDCMotor motor = HFIBLDCMotor(7,20);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOT1_A, MOT1_B, MOT1_C, MOT1_EN);
// BLDCDriver3PWM* pdriver = &driver;
LowsideCurrentSense currentsense = LowsideCurrentSense(0.01f, 50.0f, CS_A, CS_B);
// LowsideCurrentSense* pcurrentsense = &currentsense;

Commander command = Commander(Serial);
void onMotor(char* cmd){command.motor(&motor,cmd);}

void process_hfi(){motor.process_hfi();}



void setup() {
  pinMode(PC10,OUTPUT);
  Serial.begin(2000000);
	SimpleFOCDebug::enable(&Serial);
  delay(1000);

  driver.voltage_power_supply = 30;
  driver.pwm_frequency = 30000;
  // driver.voltage_limit = driver.voltage_power_supply/2;
  driver.voltage_limit = driver.voltage_power_supply/2;
  driver.init();

  command.add('M',&onMotor,"motor");

  motor.linkDriver(&driver);

  motor.linkCurrentSense(&currentsense);
  currentsense.linkDriver(&driver);
  currentsense.init();

  motor.LPF_current_d.Tf = 1/(1000*_2PI);
  motor.LPF_current_q.Tf = 1/(1000*_2PI);
  motor.torque_controller = TorqueControlType::foc_current;
  // motor.controller = MotionControlType::velocity_openloop;
  motor.controller = MotionControlType::torque;

  motor.hfi_v = 8;

  motor.init();
  motor.initFOC();
  motor.hfi_on = true;
  delay(500);

}

void loop() {
  motor.move();
  motor.loopFOC();
  command.run();
  // Serial.println(motor.electrical_angle);
  // Serial.print(motor.current_meas.d);
  // Serial.print(", ");
  // Serial.println(motor.current_meas.q);

}
