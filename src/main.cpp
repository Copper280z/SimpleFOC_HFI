#include <Arduino.h>
#include "pinout.h"
#include "SimpleFOC.h"

HFIBLDCMotor motor = HFIBLDCMotor(7,20);

#ifdef STM32F4xx
BLDCDriver3PWM driver = BLDCDriver3PWM(MOT1_A, MOT1_B, MOT1_C, MOT1_EN);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.01f, 50.0f, CS_A, CS_B);
#endif

#ifdef ARDUINO_B_G431B_ESC1
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
#endif

#ifdef ESP32
BLDCDriver3PWM driver = BLDCDriver3PWM(26, 27, 33, 12); // c-> b1
LowsideCurrentSense currentsense = LowsideCurrentSense(0.01f, 50.0f, 35, 34);
#endif

Commander command = Commander(Serial);
void onMotor(char* cmd){command.motor(&motor,cmd);}

void process_hfi(){motor.process_hfi();}



void setup() {
  #ifdef ESP32
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  pinMode(2,OUTPUT);
  #else
  pinMode(PC10,OUTPUT);
  Serial.begin(2000000);
	#endif
  SimpleFOCDebug::enable(&Serial);

  while (!Serial.available()) {}

  driver.voltage_power_supply = 30;
  driver.pwm_frequency = 20000;
  driver.voltage_limit = driver.voltage_power_supply*0.9;
  driver.init();

  command.add('M',&onMotor,"motor");

  motor.linkDriver(&driver);

  motor.linkCurrentSense(&currentsense);
  currentsense.linkDriver(&driver);
  // don't skip current sense align with sfoc shield
  // currentsense.skip_align = true;
  currentsense.init();

  motor.current_limit = 0.3f;
  motor.P_angle.P = 0.3f;
  motor.P_angle.I = 0.1f;
  motor.P_angle.D = 0.005f;
  motor.P_angle.output_ramp = 0;
  motor.LPF_angle.Tf = 0.;
  motor.LPF_current_d.Tf = 1/(2000*_2PI);
  motor.LPF_current_q.Tf = 1/(2000*_2PI);
  motor.torque_controller = TorqueControlType::foc_current;
  // motor.controller = MotionControlType::velocity_openloop;
  motor.controller = MotionControlType::torque;

  motor.hfi_v = 9;

  motor.init();
  motor.initFOC();
  
  motor.hfi_on = true;
  motor.sensor_direction = Direction::CCW;
  motor.current_setpoint.d = 0.00f;
}

uint32_t time_prev=0;
void loop() {
  uint32_t time_now = micros();
  if ((time_now-time_prev) > 1000){
    motor.move();
    time_prev = time_now;
  }

  motor.loopFOC();
  command.run();
  // Serial.println(motor.electrical_angle);
  // Serial.print(motor.current_meas.d);
  // Serial.print(", ");
  // Serial.println(motor.delta_current.q);

}
