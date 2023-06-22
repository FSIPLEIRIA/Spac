/** ==========================================
* @title:  Target Class Test Suite
* @author: João Vieira
* @date:   2023-06-14
==========================================**/

#ifndef TST_TARGET_H_
#define TST_TARGET_H_
#include <gtest/gtest.h>
#include <cmath>
#include <math.h>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <geometry_msgs/msg/detail/pose2_d__struct.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "../src/target.h"
#include "testing_utils.hpp"

#ifdef __FSIPLEIRIA_2D_ONLY__
TargetWaypoint setupAngles( nav_msgs::msg::Odometry odom, geometry_msgs::msg::Pose2D waypoint);
#else 
TargetWaypoint setupAngles( nav_msgs::msg::Odometry odom, geometry_msgs::msg::Pose waypoint);
#endif

#endif // TST_TARGET_H_