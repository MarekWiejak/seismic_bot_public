#!/usr/bin/env python

# Copyright 2021, AstroCeNT

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
# to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
# OR OTHER DEALINGS IN THE SOFTWARE.

# AstroCeNT: Particie Astrophysics Science and Technology Centre
# Agreement MAB / 2018/7 dated 03/07/2018 (Umowa MAB/2018/7 z dn. 03.07.2018)
# The project is co-financed by the European Regional Development Fund under the Intelligent Development Operational Program
# (Projekt jest współfinansowany ze środków Europejskiego Funduszu Rozwoju Regionalnego w ramach Programu Operacyjnego Inteligentny Rozwój)
# and also from the Foundation for Polish Science grant TEAM/2016-3/19.

# Author: Marek Wiejak


import rospy
import smach
import time
from smach_ros import ServiceState
from smach_ros import SimpleActionState
from smach import CBState

from actionlib.simple_action_client import GoalStatus

import robot_state_machine
from robot_state_machine import srv
import crane_state_machine
from crane_state_machine import msg
import move_base

import tic_crane
from tic_crane import msg

import raspi_gpio
from raspi_gpio import srv

import move_base_msgs
from move_base_msgs import msg

@smach.cb_interface(outcomes=['ls_engaged', 'ls_not_engaged', 'failed'])
def ls_check(ud):
    """
    Function checking if limit switches are engaged.
    Main ls: True -> engaged, False -> disengaged
    Aux ls: True -> disengaged, False -> engaged
    """
    try:
        res_main = rospy.ServiceProxy('gpio_server_node/main_upper_ls_check_state', srv.EmptyBool)()
        res_aux = rospy.ServiceProxy('gpio_server_node/aux_upper_ls_check_state', srv.EmptyBool)()
        if res_main.state == False and res_aux.state == True:
            return 'ls_not_engaged'
        elif res_main.state == False and res_aux.state == False:
            return 'ls_not_engaged'
        elif res_main.state == True and res_aux.state == False:
            return 'ls_engaged'
        elif res_main.state == True and res_aux.state == True:
            rospy.logwarn("Main limit switch signals being engaged while auxiliary limit switch signals being disengaged, resulting in forbidden state. Limit switch signal wires might be broken. Aborting measurement procedure. Beware, the sensor might still be deployed!")
            return 'failed'
    except rospy.ServiceException as e:
        rospy.logwarn("Limit switch state chceck failed. Aborting measurement procedure")
        return 'failed'

def main():
    rospy.init_node('robot_state_machine')

    # Import parameters from parameter server
    new_task_service = rospy.get_param("~new_task_service_name")
    recover_timeout = rospy.get_param("~recover_timeout")
    recover_speed = rospy.get_param("~recover_speed")

    # Prepare action goal messages for state machine
    recover_goal = tic_crane.msg.RecoverGoal(timeout = recover_timeout, speed = recover_speed)

    sm = smach.StateMachine(outcomes=['terminated', 'aborted'])
    with sm:
        def move_to_position_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                if userdata.if_measure_after_movement == True:
                    return 'measure'
                elif userdata.if_measure_after_movement == False:
                    return 'ask_next'
            elif status == GoalStatus.ABORTED:
                return 'abort'
            elif status == GoalStatus.PREEMPTED:
                return 'ask_next'

        def move_to_position_goal_cb(userdata, goal):
            return userdata.move_base_target_pose.goal

        def new_task_response_cb(userdata, response):
            userdata.if_measure_after_movement = response.if_measure_after_movement
            if response.next_step == 'measure':
                return 'measure'
            elif response.next_step == 'terminate':
                return 'terminate'
            elif response.next_step == 'move':
                tp  = move_base_msgs.msg.MoveBaseActionGoal()
                tp.header = response.goal_pose.header
                tp.goal.target_pose = response.goal_pose
                
                userdata.move_base_target_pose = tp
                return 'move'
            elif response.next_step == 'wait':
                time.sleep(5.0)
                return 'loop'


        init_sm = smach.StateMachine(outcomes=['init_succeded','init_failed'])
        with init_sm:
            init_sm.add('CHECK_IF_SENSOR_PRESENT', CBState(ls_check), transitions = {'ls_engaged':'init_succeded', 'ls_not_engaged':'RECOVER_SENSOR', 'failed':'init_failed'})
            init_sm.add('RECOVER_SENSOR', SimpleActionState('/tic_crane_node/recover', tic_crane.msg.RecoverAction, goal=recover_goal), transitions={'succeeded':'init_succeded', 'aborted':'init_failed', 'preempted':'init_failed'})
            init_sm.set_initial_state(['CHECK_IF_SENSOR_PRESENT'])

        sm.add('INITIALIZATION', 
                init_sm, 
                transitions={'init_succeded':'ASK_FOR_NEW_TASK','init_failed':'terminated'})
        sm.add('MOVE_TO_POSITION',
                SimpleActionState('/move_base',
                                move_base_msgs.msg.MoveBaseAction, 
                                goal_cb = move_to_position_goal_cb,
                                result_cb = move_to_position_result_cb, 
                                input_keys=['if_measure_after_movement','move_base_target_pose'],
                                outcomes=['measure', 'ask_next', 'abort']),
                transitions={'measure':'MEASUREMENT_PROCEDURE','ask_next':'ASK_FOR_NEW_TASK','abort':'aborted','succeeded':'terminated','preempted':'ASK_FOR_NEW_TASK'},
                remapping={'if_measure_after_movement':'if_measure_after_movement',
                            'move_base_target_pose':'move_base_target_pose'})
        sm.add('MEASUREMENT_PROCEDURE', 
                SimpleActionState('/crane_module_task_server', 
                                crane_state_machine.msg.CraneStateMachineAction), 
                transitions={'succeeded':'ASK_FOR_NEW_TASK', 'preempted':'ASK_FOR_NEW_TASK', 'aborted':'terminated'})
        sm.add('ASK_FOR_NEW_TASK',
                ServiceState(new_task_service, 
                            robot_state_machine.srv.NewTask,
                            response_cb = new_task_response_cb,
                            output_keys = ['if_measure_after_movement', 'move_base_target_pose'],
                            input_keys = ['if_measure_after_movement'],
                            outcomes=['measure', 'terminate', 'move', 'loop']),
                transitions={'terminate':'terminated','aborted':'aborted','measure':'MEASUREMENT_PROCEDURE','move':'MOVE_TO_POSITION', 'loop':'ASK_FOR_NEW_TASK', 'preempted':'terminated', 'succeeded':'terminated'},
                remapping={'if_measure_after_movement':'if_measure_after_movement',
                            'move_base_target_pose':'move_base_target_pose'})
        
        sm.set_initial_state(['INITIALIZATION'])

    sm.execute()

if __name__ == '__main__':
    main()