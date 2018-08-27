#ifndef PID_H
#define PID_H
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double prev_cte;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle
  */
  std::vector<double> dp;
  int step, param_index;
  int steps_twiddling, steps_evaluation;

  double total_error_calculating;
  double total_error;
  double best_error;

  //twiddle state flag:
  bool adding_done_flag;
  bool subtracting_done_flag;
  bool twiddle_enable;

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
  * Twiddle
  */
  void UpdateParams(int index, double param_value);


};

#endif /* PID_H */
