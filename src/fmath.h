/** ==========================================
* @title: FMath Header File
* @description: Contains generic math functions and classes
* @author: João Vieira
* @date:   2023-06-14
========================================== **/

#ifndef FMATH_H_
#define FMATH_H_
#include <cstdint>
#include "utils.h"
#include <cmath>

//#include <stdfloat>
// TODO update compiler so we can use fixed width types. 
#ifdef __FSIPLEIRIA_PID_DOUBLE_PRECISION__
	typedef long double fs_PidFloat_t;
#else
	typedef float fs_PidFloat_t;
#endif

/**
* @class PID_Controller
* @brief Provides a discrete implementation of the PID Control Algorithm
**/
class PID_Controller{
	public:
		PID_Controller(fs_PidFloat_t Kp, fs_PidFloat_t Ki, fs_PidFloat_t Kd);
		fs_PidFloat_t computeWithRateChange(fs_PidFloat_t setpoint, fs_PidFloat_t input, fs_PidFloat_t *output);
		fs_PidFloat_t compute(fs_PidFloat_t setpoint, fs_PidFloat_t input);
		int s_Tunings(fs_PidFloat_t Kp, fs_PidFloat_t Ki, fs_PidFloat_t Kd);
		fs_PidFloat_t g_Proportion();
		fs_PidFloat_t g_Integral();
		fs_PidFloat_t g_Derivative();

	protected:
		PID_Controller();
		fs_PidFloat_t kp, ki, kd;
		//error_prevtwo, calls
		fs_PidFloat_t output_past, error, error_prev, error_sum;
	
};
#ifdef __FSIPLEIRIA_KINEMATICS_DOUBLE_PRECISION__
	typedef long double fs_KinematicsFloat_t;
#else
	typedef float fs_KinematicsFloat_t;
#endif

/**
* @class PID_Controller
* @brief Provides a discrete implementation of the PID Control Algorithm
**/
class Inverse_Kinematics{
	public:
		
		Inverse_Kinematics();
		fs_KinematicsFloat_t track_ComputeSteeringAngle(fs_KinematicsFloat_t track_angle, fs_KinematicsFloat_t track_radius);
};
#endif // FMATH_H_