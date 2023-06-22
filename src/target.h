/** ==========================================
* @title: Target Class
* @class: Its the class that implements the single instance behaviour spac ought to have.
* @author: João Vieira
* @date:   2023-06-14
========================================== **/
#ifndef TARGET_H_
#define TARGET_H_
#include <cstdio>
#include <string>
#include <chrono>
#include <algorithm>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include "fmath.h"
#include <ackermann_msgs/msg/detail/ackermann_drive__builder.hpp>
#include <ackermann_msgs/msg/detail/ackermann_drive__struct.hpp>
#include <cmath>
#include <cstdio>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <math.h>
#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/logging.hpp>
#include "utils.h"
		


#ifdef __FSIPLEIRIA_2D_ONLY__
	#include <geometry_msgs/msg/pose2_d.hpp>
#else 
	#include <geometry_msgs/msg/pose.hpp>
#endif


class TargetWaypoint{
	public: 
		TargetWaypoint(float track_width);
		TargetWaypoint();
		//used for c_TargetWaypoint
		TargetWaypoint(const TargetWaypoint &target);
		
		virtual ~TargetWaypoint() = default;
		ackermann_msgs::msg::AckermannDrive m_CurrentDrive;

		fs_KinematicsFloat_t fly_Throught(fs_KinematicsFloat_t distance, fs_KinematicsFloat_t track_angle);
		fs_KinematicsFloat_t predict_trackAngle();
		fs_KinematicsFloat_t current_Angle();
		
		ackermann_msgs::msg::AckermannDrive g_dirtyDispatcherMail();
		bool g_isDispatcherDirty();
		int s_throwDirtDispatcher();
		void instance_CarrotControl();
		
		int s_TrackWidth(const float trackWidth);
		float g_TrackWidth();

		
		#ifdef __FSIPLEIRIA_2D_ONLY__
			int s_CurrentTargetWaypoint(const geometry_msgs::msg::Pose2D::SharedPtr msg);
			geometry_msgs::msg::Pose2D g_CurrentTargetWaypoint();
		#else
			int s_CurrentTargetWaypoint(const geometry_msgs::msg::Pose::SharedPtr msg);
			geometry_msgs::msg::Pose g_CurrentTargetWaypoint();
		#endif
		int s_CurrentOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
		nav_msgs::msg::Odometry g_CurrentOdometry();
	protected:
		bool m_isDispatcherDirty=true;
		ackermann_msgs::msg::AckermannDrive m_DispatcherMailBox; 
		PID_Controller* m_pid_controller;
		PID_Controller* m_pid_controller_angular;
		Inverse_Kinematics m_Inverse_Kinematics;
		float m_trackWidth=0.0f;
		fs_PidFloat_t ks_p=(fs_PidFloat_t)0.1, ks_i=(fs_PidFloat_t)0.1, ks_d=(fs_PidFloat_t)0.1;
		fs_PidFloat_t kd_p=(fs_PidFloat_t)0.1, kd_i=(fs_PidFloat_t)0.1, kd_d=(fs_PidFloat_t)0.1;
		nav_msgs::msg::Odometry m_CurrentOdometry;
		#ifdef __FSIPLEIRIA_2D_ONLY__
			geometry_msgs::msg::Pose2D m_CurrentTargetWaypoint;
		#else
			geometry_msgs::msg::Pose m_CurrentTargetWaypoint;
		#endif
		
};
#endif // TARGET_H_