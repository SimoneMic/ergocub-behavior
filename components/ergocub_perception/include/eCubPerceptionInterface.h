/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Autogenerated by Thrift Compiler (0.14.1-yarped)
//
// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_SERVICE_ECUBPERCEPTIONINTERFACE_H
#define YARP_THRIFT_GENERATOR_SERVICE_ECUBPERCEPTIONINTERFACE_H

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

class eCubPerceptionInterface :
        public yarp::os::Wire
{
public:
    // Constructor
    eCubPerceptionInterface();

    virtual std::vector<yarp::sig::Matrix> get_poses();

    virtual yarp::sig::Vector get_center();

    virtual double get_distance();

    virtual bool is_focused();

    virtual yarp::sig::Vector get_face_position();

    virtual std::int16_t get_action();

    // help method
    virtual std::vector<std::string> help(const std::string& functionName = "--all");

    // read from ConnectionReader
    bool read(yarp::os::ConnectionReader& connection) override;
};

#endif // YARP_THRIFT_GENERATOR_SERVICE_ECUBPERCEPTIONINTERFACE_H
