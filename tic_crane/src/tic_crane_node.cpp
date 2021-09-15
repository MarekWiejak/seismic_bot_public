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

#include <ros/ros.h>
#include <tic.hpp>
#include <raspi_gpio/WaitForEdgeAction.h>
#include <actionlib/client/simple_action_client.h>
#include "../include/tic_crane/motor_handle.hpp"
#include "../include/tic_crane/move_to_position_action.hpp"
#include "../include/tic_crane/recover_action.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tic_crane_node");
    ros::NodeHandle nh("~");
    
    actionlib::SimpleActionClient<raspi_gpio::WaitForEdgeAction> ac("gpio_server_node/main_upper_ls_monitor_state", false);

    tic_crane::MotorHandle motor(nh);
    tic_crane::MoveToPositionActionServer AS_mtp("move_to_target_pose", nh, &motor);
    tic_crane::RecoverActionServer AS_r("recover", nh, &motor, &ac);

    ros::spin();
    return 0;
}
