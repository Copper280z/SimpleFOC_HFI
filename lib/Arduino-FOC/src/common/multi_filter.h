#ifndef MULTI_FILTER_H
#define MULTI_FILTER_H


#include "time_utils.h"
#include "foc_utils.h"

/**
 *  Multi filter class
 *  Implements a state variable filter to compute low-, high-, bandpass and notch filters 
 *  with variable Q (resonance) in one operation, as well as seamless switching between them
 */
class MultiFilter
{
public:
    /**
     * @param Tf - Multi filter time constant
     * @param q - Filter resonance
     */
    MultiFilter(float Tf, float q=0.7f);
    MultiFilter() = default;

    enum returnType {
        MULTI_FILTER_LOWPASS,
        MULTI_FILTER_HIGHPASS,
        MULTI_FILTER_BANDPASS,
        MULTI_FILTER_NOTCH
    };

    float operator() (float x);
    float Tf; //!< Multi filter time constant

    void setQ(float newQ);  //!< Set filter resonance
    void setNotchDepth(float newNotchDepth); //!< Set notch filter depth
    void setfrequency(float newfrequency); //!< set filter frequency instead of time constant
    void setReturnType(returnType type); //!< change between low-, high-, bandpass, or notch output as default return value

    //!< Get different filter outputs
    float getLp();
    float getHp();
    float getBp();
    float getNotch();

    //!< Get different filter outputs while updating the filter state
    float getLp(float x);
    float getHp(float x);
    float getBp(float x);
    float getNotch(float x);

protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp

    float yl_prev; //!< filtered  lowpass value in previous execution step 
    float yh_prev; //!< filtered highpass value in previous execution step 
    float yb_prev; //!< filtered bandpass value in previous execution step 
    float yn_prev; //!< filtered notch value in previous execution step 

    float q = 0.7f;       //!< filter resonance.
    float notchDepth = 0.0f;       //!< notch filter cut depth.

    float alpha1;
    float alpha2 = 1.0f/q;

    returnType defaultFilter = MULTI_FILTER_LOWPASS;
};

#endif // MULTI_FILTER_H