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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tic_crane/RecoverAction.h>
#include <raspi_gpio/WaitForEdgeAction.h>
    

namespace tic_crane{

    class MotorHandle;

    /** 
        An action server class performing sensor recovery if its position is unknown.
        When action is called, the sensor will be retracted with given velocity until limit switch is reached or given timeout is reached.
        
        @param name name under which the action will be advertised
        @param nh node handle
        @param mh pointer to a motor handle
        @param ac pointer to an action server performing position detection with limit switch
    */
    class RecoverActionServer {
        protected:
            MotorHandle *mh_;
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<tic_crane::RecoverAction> as_;
            std::string action_name_;
            tic_crane::RecoverFeedback feedback_;
            tic_crane::RecoverResult result_;

        public:
            RecoverActionServer(std::string name, ros::NodeHandle nh, MotorHandle *mh, actionlib::SimpleActionClient<raspi_gpio::WaitForEdgeAction> *ac) :
                mh_(mh),
                nh_(nh),
                as_(nh_, name, boost::bind(&RecoverActionServer::executeCB, this, _1, mh_, ac), false),
                action_name_(name)
            {
                as_.start();    
            }

            ~RecoverActionServer(void){}
        
            void executeCB(const tic_crane::RecoverGoalConstPtr &goal, MotorHandle*mh, actionlib::SimpleActionClient<raspi_gpio::WaitForEdgeAction> *ac);
        };
}