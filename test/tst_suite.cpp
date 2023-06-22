/** ==========================================
* @title:  Testing Main File
* @author: João Vieira
* @date:   2023-06-14
==========================================**/

#include <gtest/gtest.h>
#include <cmath>

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>


static rclcpp::Node::SharedPtr node_;



int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	

	testing::InitGoogleTest(&argc, argv);
	int ret = RUN_ALL_TESTS();
	

	rclcpp::shutdown();
	return ret;
}
