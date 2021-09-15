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
from geometry_msgs.msg import Twist

control_vels = Twist()
control_vels.linear.x = 0
control_vels.linear.y = 0
control_vels.angular.z = 0
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def write_x_y(data):
    control_vels.linear.x = data.linear.x
    control_vels.linear.y = data.linear.y
    pub.publish(control_vels)


def write_z(data):
    control_vels.angular.z = data.angular.z
    pub.publish(control_vels)

def controller2vel():
    rospy.init_node('controller2vel')
    rospy.Subscriber('control_x_y', Twist, write_x_y)
    rospy.Subscriber('control_z', Twist, write_z)
    rospy.spin()

if __name__=='__main__':
    try:
        controller2vel()
    except rospy.ROSInterruptException:
        pass