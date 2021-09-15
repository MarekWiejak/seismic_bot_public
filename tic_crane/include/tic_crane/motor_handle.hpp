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
 */

#include <ros/ros.h>
#include <tic.hpp>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "std_srvs/Empty.h"

namespace tic_crane{

    // a class handling Pololu TIC controller - a brigde between ROS and the controller 
    class MotorHandle {
        public:
            MotorHandle(ros::NodeHandle nh);
            ~MotorHandle();
            void resetCommandTimeoutLoop();

            void setSpeed(int speed);
            void energize();
            void deenergize();
            void exitSafeStart();
            void setTargetPosition(int position);
            void setTargetVelocity(int velocity);
            void haltAndSetPosition(int position);
            void resetCommandTimeout();
            tic::variables getVariables();

        private:
            bool resetPostion(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
            tic::handle openHandle(const char * cDeviceSerialNum = nullptr);

            ros::ServiceServer reset_srv_;
            ros::NodeHandle nh_;
            tic::handle th_;
            tic::variables vars_;
            int rotation_speed_;
        };
};