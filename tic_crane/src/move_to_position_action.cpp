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

#include "../include/tic_crane/move_to_position_action.hpp"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tic_crane/MoveToPositionAction.h>
#include "../include/tic_crane/motor_handle.hpp"
#include <tic.hpp>

namespace tic_crane{

    void MoveToPositionActionServer::executeCB(const tic_crane::MoveToPositionGoalConstPtr &goal, MotorHandle *mh){
        tic::variables var_;
       
        // Initialization
        mh->setSpeed(goal->speed);
        mh->energize();
        mh->exitSafeStart();
        mh->setTargetPosition(goal->position);
        ROS_INFO("Starting Move to position action");

        // Execution - waiting untill motor reaches goal position
        do{
            mh->resetCommandTimeout();
            var_ = mh->getVariables();
            feedback_.position = var_.get_current_position();
            as_.publishFeedback(feedback_);
        }while( feedback_.position != goal->position );

        // Deenergizing motor after the task is completed
        mh->deenergize();

        // Sending status and outcome
        result_.position = feedback_.position;
        as_.setSucceeded(result_);
        return;
    }
}