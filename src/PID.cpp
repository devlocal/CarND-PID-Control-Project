#include "PID.h"

/*
* PID class
*/

PID::PID() {
  firstMeasurement_ = true;
  sumCte_ = 0;
}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

double PID::GetControlValue() {
  return -Kp * cte_ - Kd * (cte_ - prevCte_) - Ki * sumCte_;
}

void PID::UpdateError(double cte) {
  if (firstMeasurement_) {
    cte_ = cte;
    prevCte_ = cte;
    firstMeasurement_ = false;
  } else {
    prevCte_ = cte_;
    cte_ = cte;
  }

  sumCte_ += cte;
}
