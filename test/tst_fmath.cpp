/** ==========================================
* @title:  FMath Test Suite
* @author: João Vieira
* @date:   2023-06-14
==========================================**/

#include "tst_fmath.h"
#include "testing_utils.hpp"
#include "gtest/gtest.h"
#include "gtest/internal/gtest-internal.h"
#include <rclcpp/logging.hpp>

// Checks for instant exceptions/ core dumps
TEST(tst_fmath, sanity_check_steeringAngle){
	Inverse_Kinematics srk;
	fs_KinematicsFloat_t track_angle=0.0f;
	fs_KinematicsFloat_t track_width=1.0f;
	fs_KinematicsFloat_t steering_angle=srk.track_ComputeSteeringAngle(track_angle, track_width);
	EXPECT_FLOAT_EQ(FIVECUTFLOATING(steering_angle), FIVECUTFLOATING(0.0f));

}
// Checks if formula is right by comparing precalculated problem-answers
//TODO CHANGE THIS TO BE VARIOUS VALUES IN FORM OF A TABLE
TEST(tst_fmath, precalculated_steeringAngle){
	Inverse_Kinematics srk;
	fs_KinematicsFloat_t track_angle=M_PI/8;
	fs_KinematicsFloat_t track_width=3.0f;
	fs_KinematicsFloat_t steering_angle=srk.track_ComputeSteeringAngle(track_angle, track_width);
	//manual calculations 
	//1/2

	EXPECT_FLOAT_EQ(FIVECUTFLOATING(steering_angle), FIVECUTFLOATING(0.446192));

}
// Checks if the computeWithRateChange function is working
TEST(tst_fmath, rate_change_pid){
	//from 0 to 100
	const int size=10;
	PID_Controller pid(1.0f, 1.0f, 1.0f);
	fs_PidFloat_t cvs[size]; 
	fs_PidFloat_t cvr[size];
	cvs[0]=0; 
	cvr[0]=0;

	
	for (int i=0; i<size;i++) {
		auto f = i-1; 
		
		if(f>=0){
			cvr[i]=pid.computeWithRateChange(100, cvs[f], &cvs[i]);
		}else{
			cvr[i]=pid.computeWithRateChange(100, cvs[0], &cvs[i]);
		}
		
	}
	//iterate through size and see if cvs[i]-cvs[i-1] is the same as cvs[i-1}]
	for (int i=0; i<size;i++) {
		auto f = i-1; 
		auto ret = cvs[i]-cvs[f];
		if(f>=0){

			EXPECT_FLOAT_EQ(FIVECUTFLOATING(ret), FIVECUTFLOATING(cvr[i]));
		}
	}
	//EXPECT_FLOAT_EQ(FIVECUTFLOATING(f), FIVECUTFLOATING(expected));
}
//Tests if the discrete answer of pid checks up with precalculated problem-answers
TEST(tst_fmath, tabulated_pid){
	//TODO test tabulated gains/weights
	
	GTEST_SKIP_("TO IMPLEMENT - test tabulated gains/weights"); 
}
//Tests behaviour with negative pid
TEST(tst_fmath, negative_pid){
	//TODO test negative gains/weights
	GTEST_SKIP_("TO IMPLEMENT - test negative gains/weights");
}
//Checks a random input and checkpoint to test behaviour
TEST(tst_fmath, random_check_pid){
	PID_Controller pid(1.0f, 1.0f, 1.0f);
	fs_KinematicsFloat_t error=0.0f;
	//random double
	fs_KinematicsFloat_t input=0.0f;
	input = (fs_KinematicsFloat_t)rand() / (fs_KinematicsFloat_t)RAND_MAX;
	fs_KinematicsFloat_t setpoint=0.0f;
	setpoint = (fs_KinematicsFloat_t)rand() / (fs_KinematicsFloat_t)RAND_MAX;
	if(input>setpoint){
		auto k = input;
		input=setpoint;
		setpoint=k;

	}
	error = setpoint-input;
	fs_KinematicsFloat_t output=pid.compute(setpoint, input);
	fs_KinematicsFloat_t expected = 1*error+1*error+1*(error-0);
	EXPECT_FLOAT_EQ(FIVECUTFLOATING(output), FIVECUTFLOATING(expected));
}
