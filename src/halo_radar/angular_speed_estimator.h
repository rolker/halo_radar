#ifndef ANGULAR_SPEED_ESTIMATOR_H
#define ANGULAR_SPEED_ESTIMATOR_H

#include <ros/ros.h>

struct AngularSpeedEstimator
{
  double angular_speed = 0.0;
  double measured_angular_speed = 0.0;
  double prediction_error = 0.0;
  double prediction_variance = 0.0;
  double variance = 1.0;

  double measurement_variance = pow(0.045, 2.0);
  double process_noise_variance = pow(0.0015, 2.0); // allows for rpm change

  std::map<ros::Time, double> measurement_buffer;
  ros::Duration measurement_buffer_duration = ros::Duration(0.75);
  ros::Duration max_measurement_gap = ros::Duration(0.45);

  double update(ros::Time t, double angle)
  {
    if(measurement_buffer.empty() || (t > measurement_buffer.rbegin()->first && t < measurement_buffer.rbegin()->first+max_measurement_gap))
    {
      while(!measurement_buffer.empty() && measurement_buffer.begin()->first < t-measurement_buffer_duration)
        measurement_buffer.erase(measurement_buffer.begin());

      if(!measurement_buffer.empty())
      {
        // the following only works if the angle didn't wrap
        // will correct later if needed
        bool positive = angle > measurement_buffer.rbegin()->second;

        double angle_difference = angle - measurement_buffer.rbegin()->second;

        // assume angle doesn't change more than pi during max
        // time gap
        if(std::abs(angle_difference) > M_PI)
          // correct the flag due to wrapping
          positive = !positive;

        angle_difference = angle - measurement_buffer.begin()->second;
        if (positive && angle_difference < 0.0)
          angle_difference += 2.0*M_PI;
        if (!positive && angle_difference > 0.0)
          angle_difference -= 2.0*M_PI;
        
        // kalman filter

        // predict
        double prediction_variance_factor = prediction_variance/measurement_variance;
        auto estimated_variance = variance + process_noise_variance*prediction_variance_factor;

        // measurement update

        measured_angular_speed = angle_difference/(t-measurement_buffer.begin()->first).toSec();

        auto k = estimated_variance/(estimated_variance+measurement_variance);
        prediction_error = measured_angular_speed-angular_speed;
        prediction_variance = k*prediction_variance+(1-k)*prediction_error*prediction_error;
        angular_speed += k*prediction_error;
        variance = (1.0-k)*estimated_variance;
      }

      measurement_buffer[t] = angle;

    }
    else
    {
      angular_speed = 0.0;
      measured_angular_speed = 0.0;
      variance = 1.0;
      measurement_buffer.clear();
      prediction_variance = 0.0;
    }
    return angular_speed;
  }
};


#endif
