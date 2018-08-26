#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    p_error = d_error = i_error = 0.0;

    //for Twiddle
    twiddle_step = 0;
    max_twiddle_steps = 200;
    total_error = ERROR_AT_BEGIN;
    best_error = ERROR_AT_BEGIN;
}

void PID::UpdateError(double cte) {
    p_error = cte;
    d_error = cte - p_error;
    i_error += cte;

    //update total error at the end of twiddle steps
    if(twiddle_step > max_twiddle_steps){
        error_after_twiddle += pow(cte,2);
        if(twiddle_step > max_twiddle_steps*5){ //define the lengh for calculating error_after_twiddle;
            twiddle_step = 0;
            total_error = error_after_twiddle;
            error_after_twiddle = 0;
        }
    }
    twiddle_step++;
}

double PID::TotalError() {
    return total_error;
}

int PID::Twiddle_Process(double twiddle_tolerance){

    int itteration = 0;
    while(total_error > twiddle_tolerance){
        cout << "Iteration: "<< itteration << "Best error: "<< best_error <<endl;
        int i;
        for(i=0; i<sizeof(twiddle_p); i++)
            twiddle_p[i] +=  twiddle_dp[i];
            if(TotalError()<best_error){
                best_error = TotalError();
                twiddle_dp[i] *=1.1;
            }
            else{
                twiddle_p[i] -= 2 * twiddle_dp[i];
                //robot = make_robot()  //Resets the robot back to the initial position and drift.
                if(TotalError()<best_error){
                    best_error = TotalError();
                    twiddle_dp[i] *= 1.1;
                }
                else{
                    twiddle_p[i] += twiddle_dp[i];
                    twiddle_dp[i] *= 0.9;
                }
            }
        itteration++;
    }
    return TWIDDLE_END;
}
