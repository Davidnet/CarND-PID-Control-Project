#include "PID.h"
#include <math.h>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  Counter = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  Counter += 1;

}

double PID::TotalError() {
  double Total_Error  = - (Kp*p_error) - (Kd*d_error) - (Ki*i_error);
  return Total_Error;
}

void PID::Twiddle(double tol){

  double error = 0;
  double best_error = TotalError();
  double sum = std::abs(dp[0]) + std::abs(dp[1]) + std::abs(dp[2]);

  Kp = p[0];
  Ki = p[1];
  Kd = p[2];

  while(sum > tol) {
    for (int i = 2; i < 3; i++) {
      p[i] += dp[i];
      Kp = p[0];
      Ki = p[1];
      Kd = p[2];
      error = TotalError();
      if ( std::abs(error) < std::abs(best_error)) {
        best_error = error;
        dp[i] *= 1.1;
      } else {
        p[i] = p[i] - 2*dp[i];
        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
        error = TotalError();

        if (std::abs(error) < std::abs(best_error)) {
          best_error = error;
          dp[i] *= 1.1;
        } else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
}