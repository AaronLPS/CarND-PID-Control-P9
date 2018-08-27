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
    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;

    // Previous cte.
    prev_cte = 0.0;

    // Twiddling parameters
    //twiddle_enable = true;
    twiddle_enable = false;
    dp = {0.1*Kp,0.1*Kd,0.1*Ki};
    step = 1;
    param_index = 2;
    steps_twiddling = 500;
    steps_evaluation = 2000;
    total_error_calculating = 0;
    total_error = std::numeric_limits<double>::max();;

    best_error = std::numeric_limits<double>::max();
    adding_done_flag = false;
    subtracting_done_flag = false;
}

void PID::UpdateError(double cte) {
    if(step == 0){
        prev_cte = cte;
    }
    p_error = cte;
    i_error += cte;
    d_error = cte - prev_cte;
    prev_cte = cte;

    //run(robot, p) // calculate total error based on a period of steps
    if (step % (steps_twiddling + steps_evaluation) > steps_twiddling){
        total_error_calculating += pow(cte,2);
    }

    // twiddle
    if (twiddle_enable && (step % (steps_twiddling + steps_evaluation) == 0)){
        std::cout << "step: " << step << " total error: " << total_error_calculating << " best error: " << best_error << endl;
        if (total_error_calculating < best_error) {
            best_error = total_error_calculating;
            if (step !=  steps_twiddling + steps_evaluation) { //start from the second round
                dp[param_index] *= 1.1;
            }
            // process next parameter
            param_index = (param_index+1) % 3;
            adding_done_flag = false;
            subtracting_done_flag = false;
        }

        if (!adding_done_flag && !subtracting_done_flag) {
            // add dp[i] to params[i]
            UpdateParams(param_index, dp[param_index]);
            adding_done_flag = true;
        }
        else if (adding_done_flag && !subtracting_done_flag) {
            // subtract dp[i] from params[i]
            UpdateParams(param_index, -2 * dp[param_index]);
            subtracting_done_flag = true;
        }
        else {
            // set it back, reduce dp[i], move on to next parameter
            UpdateParams(param_index, dp[param_index]);
            dp[param_index] *= 0.9;
            // next parameter
            param_index = (param_index + 1) % 3;
            adding_done_flag = false;
            subtracting_done_flag = false;
        }
        total_error = total_error_calculating;
        total_error_calculating = 0;
        std::cout << "Updated P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;
    }
    step++;
}

double PID::TotalError() {
    return p_error * Kp + i_error * Ki + d_error * Kd;
}

void PID::UpdateParams(int index, double param_value) {
    if (index == 0) {
        Kp += param_value;
    }
    else if (index == 1) {
        Kd += param_value;
    }
    else if (index == 2) {
        Ki += param_value;
    }
    else {
        std::cout << "index error";
    }
}
