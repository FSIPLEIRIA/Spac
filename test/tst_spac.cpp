/** ==========================================
* @title:  Spac Operational Test Suite
* @author: João Vieira
* @date:   2023-06-14
==========================================**/

#include "tst_spac.h"
#include <cstdint>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>


/**
 * @brief An operational sanity test to check if the SpacNode class doesnt start in a weird state 
**/
TEST(tst_spac, spac_NodeSanityTest){
	//instantiate spac node
	GTEST_SKIP_( "Skipping this test for now");
	auto ff = std::make_shared<SpacNode>(); 
	
	rclcpp::spin(ff);

	//Expect SpacNode g_NodeName to return DRIVEMODEL_NODE_NAME
	EXPECT_EQ(ff->g_NodeName(), DRIVEMODEL_NODE_NAME);
	//call for the rclcpp destructor for node ff
	ff.reset();
	//check if it dies
	EXPECT_EQ(ff.use_count(), 0);

	
}
/**
* @brief An operational test to check if the node is listening to incoming random odometry messages
*/
TEST(tst_spac, spac_CheckOdometrySubscription ){
	//instantiate spac node
	auto ff = std::make_shared<SpacNode>();
	rclcpp::spin_some(ff);
	auto ff2 = std::make_shared<rclcpp::Node>("test_node");
	rclcpp::spin_some(ff2);
	auto pub = ff2->create_publisher<nav_msgs::msg::Odometry>(ff->g_OdometryTopic(), 10);

	
	for (auto i =0; i<20; i++) {
		//create a random odometry message
		nav_msgs::msg::Odometry odom;
		SpacTestSuite::generateRandomOdometry(&odom);
		//publish the message
		pub->publish(odom);

		
		//check if ff received the message
		auto waypoint = ff->c_TargetWaypoint();
		
		auto received_odom = waypoint.g_CurrentOdometry();
		//check if the received odometry is the same as the one sent
		EXPECT_EQ(received_odom, odom);


	
	}
	ff.reset();
	ff2.reset();
}

/**
* @brief Generates a random nav_msgs::msg::Odometry message

*/

void SpacTestSuite::generateRandomOdometry(nav_msgs::msg::Odometry *odom){
	
	srandom(time(NULL));
	//generate a random block of memory the size of the odom message
	//if it wasnt like this it would be too annoying and non modular
	int8_t * odom_ptr = (int8_t *) odom;
	for(unsigned long i = 0; i < sizeof(*odom); i++){
		odom_ptr[i] = random();
	}

}
