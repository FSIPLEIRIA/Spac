/** ==========================================
* @title: Spac Node Class
* @class: This class is meant to be the interface with ros2 functionalities as such it derives node, Here you will find all the boilerplate for node common node behaviour.
* @author: João Vieira
* @date:   2023-06-14
========================================== **/

#include "spac_node.h"
#include "target.h"
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/logging.hpp>

SpacNode::SpacNode() : Node(DRIVEMODEL_NODE_NAME){
	this->declare_parameter(PARAMS_FREQUENCY, 0);
	this->get_parameter(PARAMS_FREQUENCY, m_frequency);
	this->declare_parameter(PARAMS_TOPIC_WAYPOINT, "/");
	this->get_parameter(PARAMS_TOPIC_WAYPOINT, m_waypoint_topic);
	this->declare_parameter(PARAMS_TOPIC_ODOMETRY, "/");
	this->get_parameter(PARAMS_TOPIC_ODOMETRY, m_odometry_topic);
	this->declare_parameter(PARAMS_TOPIC_ACKERMANN, "/");
	this->get_parameter(PARAMS_TOPIC_ACKERMANN, m_ackermann_topic);
	this->declare_parameter(PARAMS_TRACK_WIDTH, 3.0); 
	this->get_parameter(PARAMS_TRACK_WIDTH, m_TrackWidth);
	m_target_waypoint = new TargetWaypoint(m_TrackWidth);
	//create publisher for ackermann drive
	m_ackermann_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(m_ackermann_topic, 10);

	//subscribe to topic PARAMS_TOPIC_WAYPOINT of type Pose2d
	#ifdef __FSIPLEIRIA_2D_ONLY__
		 m_waypoint_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
			m_waypoint_topic, 10, std::bind(&SpacNode::waypoint_callback, this, std::placeholders::_1));
	#else 
		 m_waypoint_sub = this->create_subscription<geometry_msgs::msg::Pose>(
			m_waypoint_topic, 10, std::bind(&SpacNode::waypoint_callback, this, std::placeholders::_1));
	#endif
	 m_odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(m_odometry_topic,10, std::bind(&SpacNode::odometry_callback, this, std::placeholders::_1));
	//TODO: some asserts should be ran through to get here as this is a critical step in operational security
	
	auto interval = std::chrono::duration<double>(1.0 / m_frequency);

	/* ToResearch: Apaparently you can only start timers on the constructor no matter what?*/
	RCLCPP_INFO(this->get_logger(), "Started carrot waypoint targeting routine on { %s }", __PRETTY_FUNCTION__ );
	this->m_timer = this->create_wall_timer(interval, [this ]()-> void { this->m_target_waypoint->instance_CarrotControl();});


	RCLCPP_INFO(this->get_logger(), "Started ackermann drive dispatch routine on { %s }", __PRETTY_FUNCTION__ );
	this->m_timer_publisher= this->create_wall_timer(interval, [this]()-> void {this->dispatchAckermannDrive();});

	



}


TargetWaypoint SpacNode::c_TargetWaypoint(){
	//clone target waypoint
	return TargetWaypoint(*this->m_target_waypoint);

}
void SpacNode::dispatchAckermannDrive(){
	if(this->m_target_waypoint->g_isDispatcherDirty()){
		//RCLCPP_INFO(this->get_logger(), "Dispatching ackermann drive on { %s }", __PRETTY_FUNCTION__); 
		this->m_ackermann_publisher->publish(this->m_target_waypoint->g_dirtyDispatcherMail());
		this->m_target_waypoint->s_throwDirtDispatcher(); 

	}
}
int SpacNode::s_Frequency(int freq){
	m_frequency=freq; 
	this->set_parameter(rclcpp::Parameter(PARAMS_FREQUENCY, m_frequency));
	return 0; 
}
int SpacNode::g_Frequency(){
	this->get_parameter(PARAMS_FREQUENCY, m_frequency);
	return m_frequency; 
}
std::string SpacNode::g_NodeName(){
	return DRIVEMODEL_NODE_NAME; 
}
std::string SpacNode::g_WaypointTopic(){
	std::string topic; 
	this->get_parameter(PARAMS_TOPIC_WAYPOINT, topic);
	return topic; 
}
std::string SpacNode::g_OdometryTopic(){
	std::string topic; 
	this->get_parameter(PARAMS_TOPIC_ODOMETRY, topic);
	return topic; 
}
std::string SpacNode::g_AckermannTopic(){
	std::string topic; 
	this->get_parameter(PARAMS_TOPIC_ACKERMANN, topic);
	return topic; 
}
float SpacNode::g_TrackWidth(){
	float track_width; 
	this->get_parameter(PARAMS_TRACK_WIDTH, track_width);
	if(track_width>0.0f){
		m_TrackWidth=track_width;
		return track_width; 	
	}
	RCLCPP_ERROR(this->get_logger(), "Track width is not valid. Check your parameters");
	return -1.0f; 
	
}
void SpacNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
	RCLCPP_DEBUG(this->get_logger(), "%f", msg.get()->pose.pose.position.x);
	m_target_waypoint->s_CurrentOdometry(msg);
}
#ifdef __FSIPLEIRIA_2D_ONLY__
	void SpacNode::waypoint_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg){
		m_target_waypoint->s_CurrentTargetWaypoint(msg);

		// TODO: some logs would be nice
	}
#else
	void SpacNode::waypoint_callback(const geometry_msgs::msg::Pose::SharedPtr msg){
		m_target_waypoint->s_CurrentTargetWaypoint(msg);
		// TODO: some logs would be nice
	}
#endif



