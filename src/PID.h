#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error = 0.0;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double last_cte = 0.0;
  double last_d_error = 0.0;

  std::chrono::milliseconds last_time;

 /*
 * Total error and Anti windup Limit
 */
 double error_max = 1.0;
 double error_min = -1.0;


    /*
    * Constructor
    */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Set Error Max and Min
   */
  void set_error_lim(double error_max, double error_min);

  PID* setKp(double Kp);

  PID* setKi(double Ki);

  PID* setKd(double Kd);

};

#endif /* PID_H */
