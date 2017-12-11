#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
   * Get the value of control signal
   */
  double GetControlValue();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

private:
  double cte_;
  double prevCte_;
  double sumCte_;
  bool firstMeasurement_;
};

#endif /* PID_H */
