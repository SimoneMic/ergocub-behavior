#pragma once


#include <behaviortree_cpp_v3/action_node.h>
#include <string>
#include <future>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/os/BufferedPort.h>
#include <RPCServerInterface.h>

using namespace BT;
using namespace std;
#include <boost/property_tree/ptree.hpp>
namespace pt = boost::property_tree;

class RobotLookAtPOI :  public SyncActionNode
{
public:
    NodeStatus tick() override;
    static PortsList providedPorts();
    RobotLookAtPOI(string name, const NodeConfiguration &nc, pt::ptree bt_config);
private:
    yarp::dev::IGazeControl& controller();
    pt::ptree bt_config;
    yarp::os::Port client_port;
    RPCServerInterface gaze_controller;
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    int none_counter;
    int none_counter_thr;
    std::string last_poi;
    const std::vector<double> m_zero_poi{1.0, 0.0, 0.8};
    const std::vector<double> m_down_poi{1.0, 0.0, 0.4};
    const std::vector<double> m_left_poi{1.0, 1.0, 0.4};
    const std::vector<double> m_right_poi{1.0, -1.0, 0.4};
    yarp::os::BufferedPort<yarp::os::Bottle> m_nav_port;
    const std::string m_nav_port_name = "/BT/gaze-sweeping:i";
};
