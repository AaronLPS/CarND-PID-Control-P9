#ifndef PID_H
#define PID_H
#include <vector>

#define TWIDDLE_PROCESSING 1
#define TWIDDLE_END 0
#define ERROR_AT_BEGIN 1000

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
  * Twiddle
  */
  double twiddle_p[3]= {0,0,0}; //P, I, D as params
  double twiddle_dp[3] = {1,1,1}; //Differential params
  int twiddle_step;
  int max_twiddle_steps;
  double error_after_twiddle; // represented by ^2 to deal with positive and negative error.
  double total_error;
  double best_error;


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
  int Twiddle_Process(double twiddle_tolerance = 0.2 );


};

#endif /* PID_H */
