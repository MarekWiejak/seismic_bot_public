#!/usr/bin/env python3

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


# This node is meant to handle all interactions between ROS and GPIO interface.
# Typically, we want an object for every pin used and one service/action for given type of interaction with given pin.

import rospy
import time
import actionlib
import raspi_gpio.msg
from raspi_gpio.srv import EmptyBool
import RPi.GPIO as GPIO

EDGE_CONFIRMATION_TIME = 0.01 # Time (in seconds) to wait for edge confirmation 

class LimitSwitch():
    """
    A class for handling hardware limit switch 
    
    ...

    Attributes
    ----------
    pin_ : int
        Number of an input pin
    name_ : string
        Name of the limit limit switch, used to create Service and Action names
    pub_ : GPIO.PUD_UP / GPIO.PUB_DOWN
        Determines whether the pin is pulled-up, or pulled-down
        (default is GPIO.PUD_UP)
    check_state_srv_ : rospy.Service
        A service server checking the state of the limit switch
    wait_for_edge_action_ : WaitForEdgeAction
        An action server class monitoring state of the limit switch untill a signal edge is detected,
        a goal message determines whether a falling or rising edge should be detected
    """

    def __init__(self, pin, name, pud = GPIO.PUD_UP):
        """Parameters
        ----------
        pin : int
            Number of an input pin
        name : string
            Name of the limit limit switch, used to create Service and Action names
        pub : GPIO.PUD_UP / GPIO.PUB_DOWN (those aliases encode int values)
            Determines whether the pin is pulled-up, or pulled-down
            (default is GPIO.PUD_UP)
        """
        self.pin_ = pin
        self.name_ = name
        self.pud_ = pud
        GPIO.setup(self.pin_, GPIO.IN, pull_up_down=self.pud_)

        # advertise 
        self.check_state_srv_ = rospy.Service("~" + self.name_ + '_check_state', EmptyBool, self.check_state)
        self.wait_for_edge_action_ = WaitForEdgeAction("~" + self.name_ + '_monitor_state', self.pin_)

        rospy.loginfo('%s binary input server started', self.name_)

    # (to be passed to a service server ONLY) A method checking state of the limit switch
    def check_state(self, req):
        return GPIO.input(self.pin_)

class WaitForEdgeAction(object):
    """
    Class providing ROS Action for detecting signal edge,
    if you are unfamiliar with ROS Actions, visit: http://wiki.ros.org/actionlib
    """

    _feedback = raspi_gpio.msg.WaitForEdgeFeedback()
    _result = raspi_gpio.msg.WaitForEdgeResult()

    def __init__(self, name, pin):
        """Parameters
        ----------
        name : string
            Name under which the action will be advertised
        pin : Number of an input pin
        """
        self._action_name = name
        self.pin_ = pin
        self.mode = False
        # self._as = actionlib.SimpleActionServer(self._action_name, raspi_gpio.msg.WaitForEdgeAction, execute_cb=self.execute_cb, auto_start=False)
        self._as = actionlib.SimpleActionServer(self._action_name, raspi_gpio.msg.WaitForEdgeAction, auto_start=False)        
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.register_goal_callback(self.goal_cb)
        self._as.start()

    def preempt_cb(self):
        # Shut down edge detection
        GPIO.remove_event_detect(self.pin_)
        self._as.set_preempted()
        rospy.loginfo("Preempting action: " + self._action_name)
        return
    
    def goal_cb(self):
        goal = self._as.accept_new_goal()
        if goal.mode == True:
            self.mode = True
            mode = GPIO.RISING
        else:
            self.mode = False
            mode = GPIO.FALLING
        rospy.loginfo("Starting action: " + self._action_name)
        self._feedback.initial_state = GPIO.input(self.pin_)

        # Real work is done here
        # Starting threaded edge detection
        GPIO.add_event_detect(self.pin_, mode, callback = self.edge_callback)
        return

    def edge_callback(self, channel):
        # To avoid interpreting signal jitter as a signal edge, the state of the switch is checked EDGE_CONFIRMATION_TIME seconds after detection
        # An example of undesired behaviour (countered by following implementation):
        # We want to detect falling edge (True --> False). The state of the switch is False, than it changes to True (rising edge),
        # rising edge should be ignored, but due to signal jitter a falling edge is detected during state transition, which is incorrect.

        # TODO: "time.sleep" is in general not a good solution for callback function as it apparently blocks intrruptions from other inputs, find different solution
        #       in practice 10ms seems to be long enough for pin state to settle and short enough not to block interruptions handling
        time.sleep(EDGE_CONFIRMATION_TIME)
        edge = GPIO.input(self.pin_)
        # Check if state of the switch is as expected, therefore confirming signal edge detection
        if edge == self.mode:
            self._result.state = edge
            self._as.set_succeeded(self._result)
            # If confirmed correct state, shut down edge detection
            GPIO.remove_event_detect(self.pin_)
            rospy.loginfo("Successfully ending action: " + self._action_name)
        return

def main():
    rospy.init_node('gpio_server')  
    GPIO.setmode(GPIO.BCM)

    # Load from parameter server numbers of Raspberry Pi pins connected to limit switches
    LIMIT_SWITCH_GPIO = rospy.get_param('~limit_switch_main_id')
    LIMIT_SWITCH_AUX_GPIO = rospy.get_param('~limit_switch_aux_id')

    # Initialize an object for each limit switch
    # WARNING: do not change names, as they are hardcoded in other parts of code
    # If you really wish to change them, check which nodes publish and subscribe relevant topics using "rostopic list" and "rostopic info /topic_name/"
    # console commands
    limit_switch = LimitSwitch(LIMIT_SWITCH_GPIO, 'main_upper_ls')
    limit_switch_aux = LimitSwitch(LIMIT_SWITCH_AUX_GPIO, 'aux_upper_ls')

    rospy.loginfo('GPIO server initialized')

    rospy.spin()

if __name__ == '__main__':
    main()

