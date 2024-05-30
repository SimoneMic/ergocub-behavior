#include <behaviortree_cpp_v3/action_node.h>

#include "robot_look_at_poi.h"
#include "common.h"

#include <chrono>
#include <thread>
#include <unistd.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <iostream>
#include <list>

using yarp::os::BufferedPort;
using yarp::os::Bottle;
using yarp::os::Property;
using yarp::dev::PolyDriver;


RobotLookAtPOI::RobotLookAtPOI(string name, const NodeConfiguration& nc, pt::ptree bt_config) :
    SyncActionNode(name, nc),
    bt_config(bt_config)
{
     yarp::os::Network yarp;
    
    std::string server_name = bt_config.get<std::string>("components.gaze.port");
    std::string client_name = "/BT/" + name + server_name;

    client_port.open(client_name);

    while (!yarp.connect(client_name,server_name))
    {
        std::cout << "Error! Could not connect to server " << server_name << '\n';
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    this->gaze_controller.yarp().attachAsClient(client_port);
    this->gaze_controller.set_gain(0.01);
    none_counter = 0;
    last_poi = "";

    m_nav_port.open(m_nav_port_name);
}

NodeStatus RobotLookAtPOI::tick()
{
    auto ms_wait = 1500ms;  // TODO parameterize
    auto data = m_nav_port.read(false);     // Read if I have to look around
    if (data != nullptr)
    {
        // data(0) = 0 for home, 1 for left, 2 for right, 3 for full sweep
        switch (data->get(0).asInt32())
        {
            // The waits should be removed and implemented a parallel wait system for sweeping gaze
            // Should return Running
        case 0:
            this->gaze_controller.look_at(m_zero_poi);
            std::this_thread::sleep_for(ms_wait);
            return NodeStatus::SUCCESS;
            break;
        case 1:
            //this->gaze_controller.look_at(m_down_poi);
            //std::this_thread::sleep_for(500ms);
            this->gaze_controller.look_at(m_left_poi);
            std::this_thread::sleep_for(ms_wait);
            //this->gaze_controller.look_at(m_down_poi);
            //std::this_thread::sleep_for(ms_wait);
            this->gaze_controller.look_at(m_zero_poi);  // is it worth doing this?
            std::this_thread::sleep_for(500ms);
            return NodeStatus::SUCCESS;
            break;
        case 2:
            //this->gaze_controller.look_at(m_down_poi);
            //std::this_thread::sleep_for(500ms);
            this->gaze_controller.look_at(m_right_poi);
            std::this_thread::sleep_for(ms_wait);
            //this->gaze_controller.look_at(m_down_poi);
            //std::this_thread::sleep_for(ms_wait);
            this->gaze_controller.look_at(m_zero_poi);  // is it worth doing this?
            std::this_thread::sleep_for(250ms);
            return NodeStatus::SUCCESS;
            break;
        case 3:
            //this->gaze_controller.look_at(m_down_poi);
            //std::this_thread::sleep_for(500ms);
            this->gaze_controller.look_at(m_left_poi);
            std::this_thread::sleep_for(ms_wait);
            //this->gaze_controller.look_at(m_down_poi);
            //std::this_thread::sleep_for(ms_wait);
            this->gaze_controller.look_at(m_right_poi);
            std::this_thread::sleep_for(2200ms);
            //this->gaze_controller.look_at(m_down_poi);
            //std::this_thread::sleep_for(ms_wait);
            this->gaze_controller.look_at(m_zero_poi);  // is it worth doing this?
            std::this_thread::sleep_for(500ms);
            return NodeStatus::SUCCESS;
            break;        
        default:
            std::cout << "[RobotLookAtPOI::tick] Doing Nothing: " << data->get(0).asInt32() << std::endl;
            break;
        }
        data->clear();
    }
    
    // Read poi type
    Optional<std::string> msg1 = getInput<std::string>("poi");
    if (!msg1)
    {
      throw BT::RuntimeError("missing required input [message]: ",
                              msg1.error() );
    }
    std::string poi = msg1.value();

    std::cout << "ROBOT LOOK AT POI: Robot says: " << msg1.value() << std::endl;

    // Declare final point
    std::vector<double> setpoint;

    // Get poi_pos if there is poi
    if(poi != "none"){
        if(last_poi != "not none"){
            this->gaze_controller.set_gain(0.01);
            last_poi = "not none";
        }
        none_counter = 0;

        Optional<std::vector<double>> msg2 = getInput<std::vector<double>>("poi_pos");
        if (!msg2)
        {
            throw BT::RuntimeError("missing required input [message]: ",
                              msg2.error() );
        }
        std::vector<double> poi_pos = msg2.value();

        // std::vector<double> poi_pos = msg2.value();
        // std::cout << poi_pos[0] << " " <<  poi_pos[1] << " " <<  poi_pos[2] << std::endl;

        setpoint.push_back(poi_pos[0]);
        setpoint.push_back(poi_pos[1]);
        setpoint.push_back(poi_pos[2]);  // offset for camera, remove it in ergoCub
        this->gaze_controller.look_at(setpoint);
    }
    else{
        if(last_poi != "none"){
            this->gaze_controller.set_gain(0.001);
            last_poi = "none";
        }
        none_counter++;
        if(none_counter > 12){
            //setpoint.push_back(1);
            //setpoint.push_back(0);
            //setpoint.push_back(0.8);
            //this->gaze_controller.look_at(setpoint);
            this->gaze_controller.look_at(m_zero_poi);
        }
    }

    return NodeStatus::SUCCESS;
}

PortsList RobotLookAtPOI::providedPorts()
{
    return {InputPort<std::string>("poi"),
            InputPort<std::vector<double>>("poi_pos")};
}
