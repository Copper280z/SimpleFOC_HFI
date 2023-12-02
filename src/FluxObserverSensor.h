#ifndef FLUX_OBSERVER_SENSOR_H
#define FLUX_OBSERVER_SENSOR_H

#include "Arduino.h"
#include "../../Arduino-FOC/src/SimpleFOC.h"
#include "common/base_classes/FOCMotor.h"
#include "common/base_classes/Sensor.h"
#include "common/multi_filter.h"
/**
  
*/

class FluxObserverSensor : public Sensor
{
  public:
    /**
    FluxObserverSensor class constructor
    @param m  Motor that the FluxObserverSensor will be linked to
    */
    FluxObserverSensor(const BLDCMotor& m);
    void update() override;
 
    void init() override;

    // Abstract functions of the Sensor class implementation
    /** get current angle (rad) */
    float getSensorAngle() override;

    
    // For sensors with slow communication, use these to poll less often
    PhaseCurrent_s current;
    DQCurrent_s estimated_dq;
    DQCurrent_s estimated_dq_prev;
    DQCurrent_s delta_dq;

    unsigned int sensor_downsample = 0; // parameter defining the ratio of downsampling for sensor update
    unsigned int sensor_cnt = 0; // counting variable for downsampling 
    float int1=0;
    float int2=0;
    float int3=0;
    float theta_err=0;

    float electrical_angle;
    float i_alpha;
    float i_beta;
    float flux_alpha = 0; // Flux Alpha 
    float flux_beta = 0; // Flux Beta
    float flux_linkage = 0; // Flux linkage, calculated based on KV and pole number
    float i_alpha_prev = 0; // Previous Alpha current
    float i_beta_prev = 0; // Previous Beta current
    float electrical_angle_prev = 0; // Previous electrical angle
    float angle_track = 0; // Total Electrical angle
    float bemf_threshold = 30.; // Bemf voltage amplitude when the flux observer should start tracking
    int8_t first = 1; // To skip angle difference calculation the first time
    float i_ah, i_bh, i_ah_prev, i_bh_prev; //Stores the band passed currents and previous difference values
    float Ts, e, e_in_prev, theta_rcal, theta_hat, theta_out_prev, wrotor, wrotor_prev, kp, ki; //PLL values
    MultiFilter filter_lpf, filter_calc_b, a_lpf, b_lpf; //Filters for HFI

  protected:    
    const BLDCMotor& _motor;

};

#endif
