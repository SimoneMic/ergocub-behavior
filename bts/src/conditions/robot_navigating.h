#pragma once


#include <behaviortree_cpp_v3/condition_node.h>
#include <string>
#include <future>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <eCubPerceptionInterface/eCubPerceptionInterface.h>

using namespace BT;
using namespace std;
#include <boost/property_tree/ptree.hpp>
namespace pt = boost::property_tree;

#if USE_ROS
#include "rclcpp/rclcpp.hpp"
//#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include <memory>

class ros_node : public rclcpp::Node{
private:
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr m_navigation_result_sub;
    int m_status;
public:
    ros_node() : rclcpp::Node("bt_ros_node_navigation"){
    m_status = 0;
    m_navigation_result_sub = this->create_subscription<action_msgs::msg::GoalStatusArray>(
                "navigate_to_pose/_action/status",
                rclcpp::SystemDefaultsQoS(),
                [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {   
                    RCLCPP_INFO( this->get_logger(), "GOAL STATUS: %i", msg->status_list.back().status);
                    m_status = msg->status_list.back().status;
                });
};


int get_status(){
    return m_status;
    /*switch (m_status)
    {
    case 0: // STATUS_UNKNOWN
        return NodeStatus::SUCCESS;
        break;
    case 1: // STATUS_ACCEPTED The goal has been accepted and is awaiting execution
        return NodeStatus::FAILURE;
        break;
    case 2: // STATUS_EXECUTING The goal is currently being executed by the action server
        return NodeStatus::FAILURE;
        break;
    case 3: // STATUS_CANCELING The client has requested that the goal be canceled 
        return NodeStatus::SUCCESS;
        break;
    case 4: // STATUS_SUCCEEDED The goal was achieved successfully by the action server
        return NodeStatus::SUCCESS;
        break;
    case 5: // STATUS_CANCELED The goal was canceled after an external request from an action client
        return NodeStatus::SUCCESS;
        break;
    case 6: // STATUS_ABORTED The goal was terminated by the action server without an external request
        return NodeStatus::SUCCESS;
        break;
    default:
        yWarning() << "[ros_node::get_status] Received an unexpected value from the navigation status: " << m_status << " Carrying on...";
        return NodeStatus::SUCCESS
        break;
    }*/
};
};



#endif

class RobotNavigating :  public ConditionNode
{
public:
    RobotNavigating(string name, const NodeConfiguration &nc, pt::ptree bt_config);
    NodeStatus tick() override;
private:
    pt::ptree bt_config;
#if USE_ROS
    ros_node m_node;
#else
    yarp::os::Network yarp;

    std::string reader_name;
    std::string writer_name;
    yarp::os::BufferedPort<yarp::os::Bottle> reader_port;
#endif
};

