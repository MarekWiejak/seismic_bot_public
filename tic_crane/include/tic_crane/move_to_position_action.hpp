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
#include <tic_crane/MoveToPositionAction.h>
    
namespace tic_crane{
    
    class MotorHandle;

    /**
        An action server class performing movement of the sensor to given position with given velocity

        @param name name under which the action will be advertised
        @param nh node handle
        @param mh pointer to a motor handle
    */
    class MoveToPositionActionServer {
        protected:
            MotorHandle *mh_;
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<tic_crane::MoveToPositionAction> as_;
            std::string action_name_;
            tic_crane::MoveToPositionFeedback feedback_;
            tic_crane::MoveToPositionResult result_;

        public:
            MoveToPositionActionServer(std::string name, ros::NodeHandle nh, MotorHandle *mh) :
                mh_(mh),
                nh_(nh),
                as_(nh_, name, boost::bind(&MoveToPositionActionServer::executeCB, this, _1, mh_), false),
                action_name_(name)
            {
                as_.start();    
            }

            ~MoveToPositionActionServer(void){}
        
            void executeCB(const tic_crane::MoveToPositionGoalConstPtr &goal, MotorHandle *mh);
        };
}