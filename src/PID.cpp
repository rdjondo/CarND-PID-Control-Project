#include "PID.h"

//using namespace std;
#include <chrono>

// ...

using namespace std::chrono;


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::UpdateError(double cte) {
    static double last_cte = 0.0;
    static milliseconds last_time = duration_cast< milliseconds >(
            system_clock::now().time_since_epoch());
    milliseconds now = duration_cast< milliseconds >(
            system_clock::now().time_since_epoch());

    double dt = ((double)(now-last_time).count())/1000;
    p_error = Kp*cte;

    // Anti windup Integrator Limit
    i_error = i_error + Ki*cte*dt;
    if(i_error>error_max) i_error=error_max;
    if(i_error<-error_max) i_error=-error_max;

    d_error = Kd*(cte-last_cte)/(dt+1e-6);
    last_cte = cte;
}

double PID::TotalError() {
    double total_error = p_error+i_error+d_error;
    if(total_error>error_max) total_error=error_max;
    if(total_error<-error_max) total_error=-error_max;
    return total_error;
}

void PID::set_error_max(double i_error_max) {
    PID::error_max = i_error_max;
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

