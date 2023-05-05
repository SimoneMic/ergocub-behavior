/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Autogenerated by Thrift Compiler (0.14.1-yarped)
//
// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_SERVICE_COMMANDINTERFACE_H
#define YARP_THRIFT_GENERATOR_SERVICE_COMMANDINTERFACE_H

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/sig/Matrix.h>

class CommandInterface :
        public yarp::os::Wire
{
public:
    // Constructor
    CommandInterface();

    virtual bool grasp();

    virtual bool is_finished();

    virtual bool perform_cartesian_action(const std::string& actionName);

    virtual bool perform_grasp_action(const std::string& actionName);

    virtual bool perform_joint_space_action(const std::string& actionName);

    virtual bool move_hands_to_pose(const yarp::sig::Matrix& leftPose, const yarp::sig::Matrix& rightPose, const double time);

    virtual bool move_joints_to_position(const std::vector<double>& position, const double time);

    virtual bool move_object_to_pose(const yarp::sig::Matrix& pose, const double time);

    virtual bool release_object();

    virtual void stop();

    virtual void shut_down();

    // help method
    virtual std::vector<std::string> help(const std::string& functionName = "--all");

    // read from ConnectionReader
    bool read(yarp::os::ConnectionReader& connection) override;
};

#endif // YARP_THRIFT_GENERATOR_SERVICE_COMMANDINTERFACE_H