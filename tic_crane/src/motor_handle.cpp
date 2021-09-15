/*
 * Copyright 2021, AstroCeNT

 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
 * OR OTHER DEALINGS IN THE SOFTWARE.

 * AstroCeNT: Particie Astrophysics Science and Technology Centre
 * Agreement MAB / 2018/7 dated 03/07/2018 (Umowa MAB/2018/7 z dn. 03.07.2018)
 * The project is co-financed by the European Regional Development Fund under the Intelligent Development Operational Program
 * (Projekt jest współfinansowany ze środków Europejskiego Funduszu Rozwoju Regionalnego w ramach Programu Operacyjnego Inteligentny Rozwój)
 * and also from the Foundation for Polish Science grant TEAM/2016-3/19.
 * 
 * Author: Marek Wiejak
 */

#include "../include/tic_crane/motor_handle.hpp"

#include <ros/ros.h>
#include <tic.hpp>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "std_srvs/Empty.h"
#include <cmath>
#include <boost/thread.hpp>

namespace tic_crane{

    MotorHandle::MotorHandle(ros::NodeHandle nh) : nh_(nh)
    {
        try{
            th_ = openHandle();
            vars_ = th_.get_variables();
            reset_srv_ = nh_.advertiseService("reset_position", &MotorHandle::resetPostion, this);
        }
        catch (const std::exception & error)
        {
            std::cerr << "Error: " << error.what() << std::endl;
        }
    }

    MotorHandle::~MotorHandle(){
    }
        
    tic::handle MotorHandle::openHandle(const char * cDeviceSerialNum)
    {
        std::vector<tic::device> list = tic::list_connected_devices();
        for (const tic::device & device : list)
        {
            if (cDeviceSerialNum && device.get_serial_number() != cDeviceSerialNum) continue;
            return tic::handle(device);
        }
        throw std::runtime_error("No device found.");
    }

    bool MotorHandle::resetPostion(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
        th_.halt_and_set_position(uint32_t(0));
        th_.deenergize();
        return true;
    }

    void MotorHandle::resetCommandTimeoutLoop(){
        while(true){
            th_.reset_command_timeout();
            boost::this_thread::interruption_point();
            sleep(0.95);
        }
        return;
    }

    void MotorHandle::setSpeed(int speed){
        th_.set_max_speed(uint32_t(speed * 10000));
        return;
    }

    void MotorHandle::energize(){
        th_.energize();
        return;
    }

    void MotorHandle::deenergize(){
        th_.deenergize();
        return;
    }

    void MotorHandle::exitSafeStart(){
        th_.exit_safe_start();
        return;
    }

    void MotorHandle::setTargetPosition(int position){
        th_.set_target_position(position);
        return;
    }

    void MotorHandle::setTargetVelocity(int velocity){
        th_.set_target_velocity(velocity);
        return;
    }

    void MotorHandle::resetCommandTimeout(){
        th_.reset_command_timeout();
        return;
    }

    void MotorHandle::haltAndSetPosition(int position){
        th_.halt_and_set_position(position);
    }

    tic::variables MotorHandle::getVariables(){
        return th_.get_variables();
    }

};
