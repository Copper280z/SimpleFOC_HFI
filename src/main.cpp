#include <Arduino.h>
#include "pinout.h"
#include "SimpleFOC.h"

// prototypes
void initFOC(HFIBLDCMotor *motor, BLDCDriver *driver, CurrentSense *currentsense);
// prototypes

HFIBLDCMotor motor_1 = HFIBLDCMotor(7,20);
HFIBLDCMotor motor_2 = HFIBLDCMotor(7,20);

#ifdef STM32F4xx
BLDCDriver3PWM driver_1 = BLDCDriver3PWM(MOT1_A, MOT1_B, MOT1_C, MOT1_EN);
LowsideCurrentSense currentsense_1 = LowsideCurrentSense(0.01f, 50.0f, CS1_A, CS1_B);

BLDCDriver3PWM driver_2 = BLDCDriver3PWM(MOT2_A, MOT2_B, MOT2_C, MOT2_EN);
LowsideCurrentSense currentsense_2 = LowsideCurrentSense(0.01f, 50.0f, CS2_A, CS2_B);
#endif

#ifdef ARDUINO_B_G431B_ESC1
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
#endif

#ifdef ESP32
#include <soc/adc_periph.h>
#include <driver/adc.h>
BLDCDriver3PWM driver = BLDCDriver3PWM(26, 27, 33, 12); // c-> b1
LowsideCurrentSense currentsense = LowsideCurrentSense(0.01f, 50.0f, 35, 34);
#endif

Commander command = Commander(Serial);
void onMotor_1(char* cmd){command.motor(&motor_1,cmd);}
void onMotor_2(char* cmd){command.motor(&motor_2,cmd);}
void target_2(char* cmd){command.scalar(&motor_2.target,cmd);}

void process_hfi(int adc_index){
  // if (adc_index==0){
  //   motor_1.process_hfi();
  //   digitalToggle(PC10);
  //   digitalToggle(PC10);
  // } else {
  //   motor_2.process_hfi();
  //   digitalToggle(PC10);
  //   digitalToggle(PC10);
  //   digitalToggle(PC10);
  //   digitalToggle(PC10);
  // }
    digitalToggle(PC10);
    digitalToggle(PC10);
    motor_1.process_hfi();
    digitalToggle(PC10);
    digitalToggle(PC10);
    motor_2.process_hfi();
    digitalToggle(PC10);
    digitalToggle(PC10);

}



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

  command.add('M',&onMotor_1,"motor1");
  command.add('N',&onMotor_2,"motor2");
  command.add('T',&target_2,"T2");

  Serial.println("Initializing motor 1");
  initFOC(&motor_1, &driver_1, &currentsense_1);
  motor_1.disable();
  Serial.println("Initializing motor 1");
  initFOC(&motor_2, &driver_2, &currentsense_2);
  
  motor_1.enable();

}

uint32_t time_prev=0;
void loop() {
  uint32_t time_now = micros();
  if ((time_now-time_prev) > 1000){
    motor_1.move();
    motor_2.move();
    time_prev = time_now;
    // noInterrupts();
    // DQCurrent_s tmp = motor_2.current_meas;
    // interrupts();
    // Serial.printf("%.3f\n", tmp.d);
  }

  motor_1.loopFOC();
  motor_2.loopFOC();
  command.run();

}

void initFOC(HFIBLDCMotor *motor, BLDCDriver *driver, CurrentSense *currentsense) {
  driver->voltage_power_supply = 33;
  driver->pwm_frequency = 20000;
  driver->voltage_limit = driver->voltage_power_supply*0.9;
  driver->init();


  motor->linkDriver(driver);

  motor->linkCurrentSense(currentsense);
  currentsense->linkDriver(driver);
  // don't skip current sense align with sfoc shield
  // currentsense.skip_align = true;
  currentsense->init();

  motor->current_limit = 0.3f;
  motor->P_angle.P = 0.3f;
  motor->P_angle.I = 0.1f;
  motor->P_angle.D = 0.005f;
  motor->P_angle.output_ramp = 0;
  motor->LPF_angle.Tf = 0.;
  motor->LPF_current_d.Tf = 1 / (2000 * _2PI);
  motor->LPF_current_q.Tf = 1 / (2000 * _2PI);
  motor->torque_controller = TorqueControlType::foc_current;
  motor->controller = MotionControlType::torque;

  // 2804 140kv
  motor->Ld = 2200e-6f;
  motor->Lq = 3100e-6f;
  
  //5208 80kv
  // motor.Ld = 4000e-6f;
  // motor.Lq = 6500e-6f;

  motor->hfi_v = 12;

  motor->init();
  motor->initFOC();
  
  motor->hfi_on = true;
  motor->sensor_direction = Direction::CCW;
  motor->current_setpoint.d = 0.00f;
}