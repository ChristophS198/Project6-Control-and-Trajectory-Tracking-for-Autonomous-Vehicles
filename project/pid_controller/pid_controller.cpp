/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <limits>    // Christoph: Included to init cte with inf and therefore detect init state during first update round

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  cte = std::numeric_limits<double>::infinity();
  diff_cte = 0;
  sum_cte = 0;

  Kp = Kpi;
  Kd = Kdi;
  Ki = Kii;

  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  // In the first update step this-> cte is set to cte -> differential portion is zero 
  if (this->cte == std::numeric_limits<double>::infinity()) {
     this->cte = cte;
  }
  
  // For differential part the special case of delta_time == 0 must be considered
  if (delta_time == 0) {
   diff_cte = 0;
  }
  else {
   diff_cte = (cte - this->cte)/delta_time;
  }

  // Approximate integration 
  // sum_cte += cte*delta_time;
  sum_cte += delta_time*(this->cte - cte)*0.5;
  this->cte = cte;   // Update cte last

}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
    // Calculate control value based on errors
    control = -Kp*cte - Kd*diff_cte - Ki*sum_cte;

    // Ensure output is in the defined range
    control = max(control, output_lim_min);
    control = min(control, output_lim_max);
    std::cout << "Cte " << cte << " Control: " << control << " P: " << -Kp * cte << " I: " << - Ki * sum_cte << " D: " << - Kd * diff_cte << " T: " << delta_time << std::endl;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
}