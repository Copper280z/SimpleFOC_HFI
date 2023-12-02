#include "Arduino.h"
// #include "WSerial.h"
#include "communication/Commander.h"
#include "drivers/BLDCDriver3PWM.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "FluxObserverSensor.h"
#include "SimpleFOC.h"
// #include "SimpleFOCDrivers.h"
#include "variant_B_G431B_ESC1.h"

#define RPHASE 0.89
#define L_PHASE 0.62
#define PP 4
#define KV 333

// Stepper motor instance
BLDCMotor motor = BLDCMotor(PP, RPHASE, KV, L_PHASE);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// encoder instance
STM32HWEncoder encoder = STM32HWEncoder(2048, A_ENCODER_A, A_ENCODER_B);
// STM32HWEncoder encoder = STM32HWEncoder(2048, PB6_ALT2, PB7_ALT2);
FluxObserverSensor hfi = FluxObserverSensor(motor); 

//Current sensing 
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// commander interface
Commander command = Commander(Serial, '\n', true);
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimitCurrent(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

int t0=0;
long i=0;

void setup() {
    Serial.begin(115200);
    // init step and dir pins
    Serial.println("Starting HW Encoder");
    // initialize encoder sensor hardware
    encoder._pinA = PB_6_ALT2;
    encoder._pinB = PB_7_ALT2;
    encoder.init();

    Serial.println("Linking Sensor to Motor");
    hfi.init();

    // link the motor to the sensor
    motor.linkSensor(&hfi);
    // motor.linkSensor(&encoder);
    motor.hfi_enabled=true;
    motor.hfi_voltage = .5;
    motor.hfi_frequency=4000;

    // choose FOC modulation
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // motor.torque_controller = TorqueControlType::foc_current;
    motor.torque_controller = TorqueControlType::foc_current;
    motor.voltage_sensor_align = 1; // default 3V

    // power supply voltage [V]
    driver.voltage_power_supply = 12;
    driver.pwm_frequency = 35000;
    driver.init();
    currentSense.linkDriver(&driver);
    // link the motor to the sensor
    motor.linkDriver(&driver);
    currentSense.init();
    // currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    // set control loop type to be used
    motor.controller = MotionControlType::torque;

    // controller configuration based on the control type 
    motor.PID_velocity.P = 0.11;
    motor.PID_velocity.I = 1.5;
    motor.PID_velocity.D = 0.000;
    // motor.PID_velocity.P = 0.11;
    // motor.PID_velocity.I = 0.0;
    // motor.PID_velocity.D = 0.000;
    motor.LPF_velocity.Tf = 0.015;
    motor.PID_velocity.output_ramp = 0;
    // default voltage_power_supply
    motor.voltage_limit = 2;
    motor.current_limit = 1.0;
    // motor.phase_resistance = 9.9;
    // motor.phase_inductance = 0.01252;
    motor.motion_downsample = 2;
    // motor.KV_rating = 40;

    // angle loop controller
    motor.P_angle.P = 175;
    motor.P_angle.I = 1;
    motor.P_angle.D = 0.1;
    // motor.P_angle.P = 8;
    // motor.P_angle.I = 0;
    // motor.P_angle.D = 0.0;
    motor.LPF_angle = 0.00;
    // angle loop velocity limit
    motor.velocity_limit = 10000;

    // use monitoring with serial for motor init
    // monitoring port
    // comment out if not needed
    motor.useMonitoring(Serial);

    // initialise motor
    motor.init();
    // align encoder and start FOC
    motor.setPhaseVoltage(3,0,0);
    _delay(500);
    motor.sensor_offset = 1.57;
    motor.sensor_direction = Direction::CW;
    motor.initFOC();

    motor.setPhaseVoltage(0,0,0);


    // set the initial target value
    motor.target = 0;

    // define the motor id
    char motor_id = 'M';
    command.add(motor_id, onMotor, (char*) "motor");

    //For the web interface
    motor.monitor_downsample = 500;
    // motor.monitor_start_char = motor_id; // the same latter as the motor id in the commander 
    // motor.monitor_end_char = motor_id; // the same latter as the motor id in the commander
    command.verbose = VerboseMode::user_friendly;
    // commander.add('C',onPid,"my pid");
    // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
    Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));

    _delay(2000);
}

void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  motor.move();

  encoder.update();
  hfi.update();

  // if ((micros()-t0) > 1e4) {
  //     Serial.printf("%d loops per sec\n", i*100);
  //     i=0;
  //     t0 = micros();
  // }

  i+=1;

  if ((micros()-t0) > 5e3) {
    // Serial.print("t:");
    // Serial.print(motor.target);
    // Serial.print(", a:");
    // Serial.print(motor.shaftAngle());
    // Serial.print(", hfi_a:");
    // Serial.println(hfi.angle_track);
    // Serial.flush();
    // Serial.print("v:");
    // Serial.print(70*motor.shaftVelocity());
    // Serial.print(", vs:");
    // Serial.print(70*motor.shaft_velocity_sp);
    // Serial.print(", ep:");
    // Serial.print(7000*(motor.shaft_angle_sp-motor.shaft_angle));
    // Serial.print(", ev:");
    // Serial.print(70*(motor.shaft_velocity_sp-motor.shaftVelocity()));
    // Serial.print(", eq:");
    // Serial.println(100*motor.current.q);

    t0 = micros();
  }

  command.run();
  // motor.monitor();
}

