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

#include "../include/tic_crane/recover_action.hpp"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tic_crane/MoveToPositionAction.h>
#include "../include/tic_crane/motor_handle.hpp"
#include <tic.hpp>
#include <raspi_gpio/WaitForEdgeAction.h>


namespace tic_crane{

    void RecoverActionServer::executeCB(const tic_crane::RecoverGoalConstPtr &goal, MotorHandle *mh, actionlib::SimpleActionClient<raspi_gpio::WaitForEdgeAction> *ac){
        ac->waitForServer();

        raspi_gpio::WaitForEdgeGoal goal_;
        // Corresponding limit switch is a pull-up normally closed option
        // "goal_.mode = true" determines, that rising signal edge is expected
        goal_.mode = true;
        ac->sendGoal(goal_);

        bool finished_before_timeout = false;
        // Start cyclic motor power timeout reset
        boost::thread thrd(&MotorHandle::resetCommandTimeoutLoop, mh);

        mh->energize();
        mh->exitSafeStart();
        mh->setTargetVelocity(uint32_t(goal->speed * 10000));
        ROS_INFO("Starting recovery action");

        finished_before_timeout = ac->waitForResult(ros::Duration(goal->timeout));
        thrd.interrupt();

        mh->haltAndSetPosition(0);
        mh->deenergize();

        ac->cancelAllGoals();
        ac->stopTrackingGoal();
        if (finished_before_timeout)
        {
            result_.state = true;
            as_.setSucceeded(result_);
        }
        else
        {
            result_.state = false;
            ROS_WARN("Failed to recover sensor before timeout");
            as_.setAborted(result_);
        }
        return;
    }

}