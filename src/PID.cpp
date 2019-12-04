#include "PID.h"
#include <iostream>
#include <cmath>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
	Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
	
    d_error = 0;
	p_error = 0;
	i_error = 0;
	
	/*Intialize vectors for Twiddle*/
	dp = {0.1*Kp,0.1*Kd,0.1*Ki};
	run_count = 1;
    index = 2;
    n_settle = 100;
    n_train = 2000;
	
	TwiddleAlgorithm = false;
	TwiddlePart2=false;
	TwiddlePart1=false;	
	
	best_err = 100000;
	total_err = 0;
	}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

}

double PID::TotalError() {
    return (Kp * p_error + Kd * d_error + Ki * i_error);    
}


void PID::Twiddle(double cte)
{
    total_err += pow(cte,2);

    if (TwiddleAlgorithm){
        cout << "run_count: " << run_count << endl;
        cout << "total error: " << total_err << endl;
        cout << "best error: " << best_err << endl;
        if (total_err < best_err) {
            cout << "improvement!" << endl;
            best_err = total_err;
            dp[index] *= 1.1;
            index = (index + 1) % 3;
            TwiddlePart1 = false;
			TwiddlePart2 = false;
        }
        if (!TwiddlePart1 && !TwiddlePart2) {
			if (index == 0) {
				Kp += dp[index];
			}
			else if (index == 1) {
				Kd += dp[index];
			}
			else if (index == 2) {
				Ki += dp[index];
			}
            TwiddlePart1 = true;
        }
        else if (TwiddlePart1 && !TwiddlePart2) {
			if (index == 0) {
				Kp += (-2*dp[index]);
			}
			else if (index == 1) {
				Kd += (-2*dp[index]);
			}
			else if (index == 2) {
				Ki += (-2*dp[index]);
			}    
            TwiddlePart2 = true;         
        }
        else {
			 if (index == 0) {
				Kp += dp[index];
			}
			else if (index == 1) {
				Kd += dp[index];
			}
			else if (index == 2) {
				Ki += dp[index];
			}  
            dp[index] *= 0.9;
            // next parameter
            index = (index + 1) % 3;
            TwiddlePart1 = false;
			TwiddlePart2 = false;
        }
        total_err = 0;
        cout << "new parameters" << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;        
    }
    run_count++;

}