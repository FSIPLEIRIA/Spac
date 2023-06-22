/** ==========================================
* @title:  Spac Operational Test Suite
* @author: João Vieira
* @date:   2023-06-14
==========================================**/

#ifndef TST_SPAC_H_
#define TST_SPAC_H_
#include <memory>
#include <gtest/gtest.h> 
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include "../src/spac_node.h"

class SpacTestSuite {
	public:
		SpacTestSuite()= default;
		~SpacTestSuite() = default; 
		static void generateRandomOdometry(nav_msgs::msg::Odometry * odom);
};
 

#endif // TST_SPAC_H_ 