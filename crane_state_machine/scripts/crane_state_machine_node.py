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
from smach_ros import SimpleActionState
from smach_ros import ServiceState
from smach_ros import IntrospectionServer
from smach_ros import ActionServerWrapper
from smach import CBState
from std_srvs.srv import Empty
import tic_crane
import raspi_gpio
import crane_state_machine
import sensor_server
from tic_crane import msg
from raspi_gpio import msg, srv
from crane_state_machine import msg
from sensor_server import msg

# calls a sevrice checking limit switch state
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

@smach.cb_interface(outcomes=['succeeded', 'failed'])
def reset_position(ud):
    try:
        res = rospy.ServiceProxy('tic_crane_node/reset_position', Empty)()
        return 'succeeded'
    except rospy.ServiceException as e:
        print('Service call failed: %s' %e)
        return 'failed'
    

def main():
    rospy.init_node('tic_crane_state_machine')

    # Import parameters from parameter server
    deploy_position = rospy.get_param("~deploy_position")
    deploy_speed = rospy.get_param("~deploy_speed")
    store_position = rospy.get_param("~store_position")
    store_speed = rospy.get_param("~store_speed")
    recover_timeout = rospy.get_param("~recover_timeout")
    recover_speed = rospy.get_param("~recover_speed")
    fake_measurement_time = rospy.get_param("~fake_measurement_time")

    measurement_topic = rospy.get_param("~measurement_action_name")

    # Prepare action goal messages for state machine
    recover_goal = tic_crane.msg.RecoverGoal(timeout = recover_timeout, speed = recover_speed)
    deploy_goal = tic_crane.msg.MoveToPositionGoal(position = deploy_position, speed = deploy_speed)
    store_goal = tic_crane.msg.MoveToPositionGoal(position = store_position, speed = store_speed)
    measure_goal = sensor_server.msg.MeasurementGoal(measurement_duration = fake_measurement_time)
    monitor_goal = raspi_gpio.msg.WaitForEdgeAction()

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    with sm:
        sm.add('INITIALIZATION', 
                CBState(ls_check), 
                transitions = {'ls_engaged':'RESET_BEFORE_DEPLOY', 'ls_not_engaged':'RECOVER_1', 'failed':'aborted'})

        sm.add('DEPLOY',
                SimpleActionState('/tic_crane_node/move_to_target_pose', tic_crane.msg.MoveToPositionAction, goal=deploy_goal), 
                transitions = {'succeeded':'CHECK_DEPLOYED'})
        sm.add('CHECK_DEPLOYED', 
                CBState(ls_check), 
                transitions = {'ls_engaged':'aborted', 'ls_not_engaged':'MEASURE', 'failed':'aborted'})

        sm.add('MEASURE', 
                SimpleActionState(measurement_topic, sensor_server.msg.MeasurementAction, goal=measure_goal), 
                transitions={'succeeded':'STORE', 'preempted':'STORE'})

        sm.add('STORE', 
                SimpleActionState('/tic_crane_node/move_to_target_pose', tic_crane.msg.MoveToPositionAction, goal=store_goal), 
                transitions={'succeeded':'CHECK_STORED'})
        sm.add('CHECK_STORED', 
                CBState(ls_check), 
                transitions = {'ls_engaged':'succeeded', 'ls_not_engaged':'RECOVER_2', 'failed':'aborted'})

        # Sensor recovery before measurement
        sm.add('RECOVER_1', 
                SimpleActionState('/tic_crane_node/recover', tic_crane.msg.RecoverAction, goal=recover_goal), 
                transitions={'succeeded':'RESET_BEFORE_DEPLOY', 'aborted':'aborted'})
        sm.add('RESET_BEFORE_DEPLOY', 
                CBState(reset_position), 
                transitions = {'succeeded':'DEPLOY', 'failed':'aborted'})
        # Sensor recovery after measurement 
        sm.add('RECOVER_2', 
                SimpleActionState('/tic_crane_node/recover', tic_crane.msg.RecoverAction, goal=recover_goal), 
                transitions = {'succeeded':'succeeded', 'aborted':'aborted'})

        sm.set_initial_state(['INITIALIZATION'])

    # Wrap the state machine into a ROS Action
    asw = ActionServerWrapper(
        'crane_module_task_server', crane_state_machine.msg.CraneStateMachineAction, wrapped_container = sm, 
        succeeded_outcomes = ['succeeded'],
        aborted_outcomes = ['aborted'],
        preempted_outcomes = ['preempted'] )

    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()
