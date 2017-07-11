
//using namespace std;
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
#include "PID.h"

// ...



/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    last_time  = duration_cast< milliseconds >(
        system_clock::now().time_since_epoch());
}

void PID::UpdateError(double cte) {

    milliseconds now = duration_cast< milliseconds >(
            system_clock::now().time_since_epoch());

    double dt = ((double)(now-last_time).count())/1000.0;
    last_time = now;
    p_error = Kp*cte;

    // Anti windup Integrator Limit
    i_error = i_error + Ki*cte*dt;
    if(i_error>error_max) i_error=error_max;
    if(i_error<error_min) i_error=error_min;

    if (dt>1e-6) {
        d_error = Kd * (cte - last_cte) / dt;
    } else {
        d_error = 0.0;
    }

    /* Limit excessive derivative values */
    if(d_error>error_max) d_error=error_max;
    if(d_error<error_min) d_error=error_min;

    d_error = 0.4 * d_error + 0.6 * last_d_error; // Filter out derivative error to reduce instabilities
    last_d_error = d_error;
    last_cte = cte;
}

double PID::TotalError() {
    double total_error = p_error+i_error+d_error;
    if(total_error>error_max) total_error=error_max;
    if(total_error<error_min) total_error=error_min;
    return total_error;
}

void PID::set_error_lim(double error_max, double error_min) {
    PID::error_max = error_max;
    PID::error_min = error_min;
}

PID* PID::setKp(double Kp) {
    PID::Kp = Kp;
    return this;
}

PID* PID::setKi(double Ki) {
    PID::Ki = Ki;
    return this;
}

PID* PID::setKd(double Kd) {
    PID::Kd = Kd;
    return this;
}

