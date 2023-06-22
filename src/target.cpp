/** ==========================================
* @title: Target Class
* @class: Its the class that implements the single instance behaviour spac ought to have.
* @author: João Vieira
* @date:   2023-06-14
========================================== **/

#include "target.h"
#include "fmath.h"
#include <cmath>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/logging.hpp>



TargetWaypoint::TargetWaypoint(){
	m_pid_controller= new PID_Controller(ks_p, ks_i, ks_d);
	m_pid_controller_angular= new PID_Controller(kd_p, kd_i, kd_d);

}
TargetWaypoint::TargetWaypoint(float track_width):TargetWaypoint(){
	s_TrackWidth(track_width);
}
TargetWaypoint::TargetWaypoint(const TargetWaypoint &target){
	m_CurrentTargetWaypoint = target.m_CurrentTargetWaypoint;
	m_CurrentOdometry = target.m_CurrentOdometry;
	m_CurrentDrive = target.m_CurrentDrive;
	m_trackWidth = target.m_trackWidth;
	m_pid_controller = target.m_pid_controller;
	m_pid_controller_angular = target.m_pid_controller_angular;
}
//I assumed the car is naturally pointed at the +Y axis 
fs_KinematicsFloat_t TargetWaypoint::predict_trackAngle(){
	#ifdef __FSIPLEIRIA_2D_ONLY__
		
		fs_KinematicsFloat_t delta_x = m_CurrentTargetWaypoint.x - m_CurrentOdometry.pose.pose.position.x;
		fs_KinematicsFloat_t delta_y = m_CurrentTargetWaypoint.y - m_CurrentOdometry.pose.pose.position.y;
		

		// Due to above x and y is flipped here
		fs_KinematicsFloat_t theta_dot = std::atan2(delta_x,delta_y);

		return theta_dot;

	#else
		fs_KinematicsFloat_t delta_x = m_CurrentTargetWaypoint.x - m_CurrentOdometry.pose.pose.position.x;
		fs_KinematicsFloat_t delta_y = m_CurrentTargetWaypoint.y - m_CurrentOdometry.pose.pose.position.y;
		fs_KinematicsFloat_t delta_z = m_CurrentTargetWaypoint.z - m_CurrentOdometry.pose.pose.position.z;
		
		//fs_KinematicsFloat_t theta_pos = std::atan2(,);
		return 0;
	#endif

}
/**
* @brief This @b unpure function is written in a spaggeti script style and is meant to initiate pure pursuit/carrot and stick method
* @details This function is meant to be called in a asynchronous loop and it will return the steering angle to be applied to the vehicle
*/
void TargetWaypoint::instance_CarrotControl(){
	//RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "TargetWaypoint::instance_CarrotControl() called");
	//return;c
	try{

		auto theta_track = predict_trackAngle(); 
		auto theta_steer = m_Inverse_Kinematics.track_ComputeSteeringAngle(theta_track, m_trackWidth);
		auto theta_current = current_Angle(); 
		//angle that is supposed to be applied to the abstract "steering wheel"
		auto imperative_angle = m_pid_controller_angular->compute(theta_steer, theta_current);
		//clamp imperative angle to -MAX_STEERING and MAX_STEERING
		imperative_angle = std::clamp(imperative_angle, (fs_PidFloat_t)-MAX_STEERING,(fs_PidFloat_t) MAX_STEERING);
		
		
		
		#ifdef __FSIPLEIRIA_2D_ONLY__
			fs_KinematicsFloat_t __distance = std::sqrt(std::pow(m_CurrentTargetWaypoint.x - m_CurrentOdometry.pose.pose.position.x,2) + std::pow(m_CurrentTargetWaypoint.y - m_CurrentOdometry.pose.pose.position.y,2));
		
		#else
			//TODO bellow was copy pasted
			fs_KinematicsFloat_t distance = std::sqrt(std::pow(m_CurrentTargetWaypoint.x - m_CurrentOdometry.pose.pose.position.x,2) + std::pow(m_CurrentTargetWaypoint.y - m_CurrentOdometry.pose.pose.position.y,2));
		#endif
		auto speed = fly_Throught(__distance, theta_track);
		m_pid_controller->compute(speed, m_CurrentOdometry.twist.twist.linear.x);
		speed = std::clamp(speed, (fs_PidFloat_t)-TERMINAL_SPEED,(fs_PidFloat_t) TERMINAL_SPEED);
		


		m_DispatcherMailBox = ackermann_msgs::msg::AckermannDrive();
		m_DispatcherMailBox.speed = speed;
		m_DispatcherMailBox.steering_angle = imperative_angle;
		
		//for now lets keep it simple, this makes it as fast as possible
		m_DispatcherMailBox.steering_angle_velocity = 0.0f;

		
		m_isDispatcherDirty = true;


	}catch(...){
		// Makes shure the dispatcher wont look for bad data
		m_isDispatcherDirty = false; 
		
		//TODO: Log error
	
	}
}

fs_KinematicsFloat_t TargetWaypoint::fly_Throught(fs_KinematicsFloat_t distance, fs_KinematicsFloat_t track_angle){
	//this is just to deal with the PID 
	std::clamp(distance, (fs_PidFloat_t)-1*DISTANCE_MAX, (fs_PidFloat_t)DISTANCE_MAX); 
	return distance * std::pow(std::cos(track_angle),2);
	//cosine track angle * distance
}
ackermann_msgs::msg::AckermannDrive TargetWaypoint:: g_dirtyDispatcherMail(){
	//This may look "optimizable" but the reason its like this is to keep a error by default approach 
	if(m_isDispatcherDirty){
	
		return m_DispatcherMailBox;
	}
	//Log warning that the dispatcher tried to read clean data
	RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Dispatcher is trying to read clean data, this means that the dispatcher is trying to read data that has not been updated yet");
	return m_DispatcherMailBox;
} 
bool TargetWaypoint::g_isDispatcherDirty(){
	return m_isDispatcherDirty;
}
int TargetWaypoint::s_throwDirtDispatcher(){
	if(!m_isDispatcherDirty){
		//TODO: Log warning
		RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Dispatcher is dirty, this means that the dispatcher is trying to send bad data");
		return -1;
	}
	m_isDispatcherDirty = false;
	return 0;
}
fs_KinematicsFloat_t TargetWaypoint::current_Angle(){
	geometry_msgs::msg::Quaternion quaternion_pose = m_CurrentOdometry.pose.pose.orientation;
	//grab the quaternion from the odometry message convert it to euler angles
	tf2::Quaternion q(quaternion_pose.x, quaternion_pose.y, quaternion_pose.z, quaternion_pose.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;

}

int TargetWaypoint::s_TrackWidth(float track_width){
	if(track_width>0){
		m_trackWidth=track_width;
		return 0;
	}else{
		//Throw error, this is critical and cannot be ignored especially since it will most likely occur before the car even starts moving
		throw std::invalid_argument("Supplied track width has an invalid value");		
	}
	return -1;
	
}
float TargetWaypoint::g_TrackWidth(){
	return m_trackWidth;
}


#ifdef __FSIPLEIRIA_2D_ONLY__
	int TargetWaypoint::s_CurrentTargetWaypoint(const geometry_msgs::msg::Pose2D::SharedPtr msg){
		
		m_CurrentTargetWaypoint=*msg;
		return 0;
	}
	geometry_msgs::msg::Pose2D TargetWaypoint::g_CurrentTargetWaypoint(){
		return m_CurrentTargetWaypoint;
	}

#else
	int TargetWaypoint::s_CurrentTargetWaypoint(const geometry_msgs::msg::Pose::SharedPtr msg){
		m_CurrentTargetWaypoint=*msg;
		return 0;
	}
	geometry_msgs::msg::Pose TargetWaypoint::g_CurrentTargetWaypoint(){
		return m_CurrentTargetWaypoint;
	}
#endif
int TargetWaypoint::s_CurrentOdometry(const nav_msgs::msg::Odometry::SharedPtr msg){
	
	m_CurrentOdometry=*msg;

	return 0;
}
nav_msgs::msg::Odometry TargetWaypoint::g_CurrentOdometry(){
	return m_CurrentOdometry;
}
