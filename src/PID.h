#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

 /*
 * Total error and Anti windup Limit
 */
 double error_max = 1.0;


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
   * Set Error Max
   */
  void set_error_max(double i_error_max);

  PID* setKp(double Kp);

  PID* setKi(double Ki);

  PID* setKd(double Kd);

};

#endif /* PID_H */
