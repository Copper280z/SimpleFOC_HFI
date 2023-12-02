#include "multi_filter.h"

MultiFilter::MultiFilter(float time_constant, float q_factor)
    : Tf(time_constant)
    , q(q_factor)
    , yl_prev(0.0f)
    , yh_prev(0.0f)
    , yb_prev(0.0f)
    , yn_prev(0.0f)

{
    timestamp_prev = _micros();
}


float MultiFilter::operator() (float x)
{
    // Implements a state variable filter
    unsigned long timestamp = _micros();
    float dt = (timestamp - timestamp_prev)*1e-6f;

    if (dt < 0.0f ) dt = 1e-3f;
    else if(dt > 0.3f) {
        yl_prev = x;
        yh_prev = x;
        yb_prev = x;
        yn_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    alpha1 = 2 * _sin(dt/(_PI*Tf));
    float yh = x - yl_prev - alpha2 * yb_prev;
    float yb = alpha1 * yh + yb_prev;
    float yl = alpha1 * yb + yl_prev;
    yl_prev = yl;
    yh_prev = yh;
    yb_prev = yb;
    yn_prev = yl_prev + yh_prev + x * notchDepth;
    timestamp_prev = timestamp;
    
    switch (defaultFilter)
    {
    case MULTI_FILTER_LOWPASS:
        return yl_prev;
        break;
    case MULTI_FILTER_HIGHPASS:
        return yh_prev;
        break;
    case MULTI_FILTER_BANDPASS:
        return yb_prev/q;
        break;
    case MULTI_FILTER_NOTCH:
        return yn_prev/(1+notchDepth);
        break;
    default:
        return yl_prev;
        break;
    }
}

float MultiFilter::getLp() {return yl_prev;}
float MultiFilter::getHp() {return yh_prev;}
float MultiFilter::getBp() {return yb_prev/q;}
float MultiFilter::getNotch() {return yn_prev/(1+notchDepth);}

float MultiFilter::getLp(float x) 
{
    (*this)(x);  // Call operator() on current instance of MultiFilter
    return yl_prev;
}
float MultiFilter::getHp(float x) 
{
    (*this)(x);  // Call operator() on current instance of MultiFilter
    return yh_prev;
}
float MultiFilter::getBp(float x) 
{
    (*this)(x);  // Call operator() on current instance of MultiFilter
    return yb_prev/q;
}
float MultiFilter::getNotch(float x) 
{
    (*this)(x);  // Call operator() on current instance of MultiFilter
    return yn_prev/(1+notchDepth);
}

void MultiFilter::setQ(float newQ)
{
    if (newQ != 0.0f && newQ != -0.0f)
    {
        alpha2 = 1.0f / newQ;
        q = newQ;
    }else
    {
        alpha2 = 1.0f / 0.7f;
        q = 0.7f;
    }
}

void MultiFilter::setNotchDepth(float newNotchDepth)
{
    if (newNotchDepth >= 0.0f)
    {
        notchDepth = newNotchDepth;
    }else
    {
        notchDepth = 0.0f;
    }
}

void MultiFilter::setfrequency(float newfrequency)
{
    if (newfrequency > 0.0f)
    {
        Tf = 1.0f / newfrequency;
    }else
    {
        Tf = 1e-3f;
    }
}

void MultiFilter::setReturnType(returnType type)
{
    defaultFilter = type;
}
