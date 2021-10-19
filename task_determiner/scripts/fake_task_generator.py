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

from geometry_msgs.msg import PoseStamped
from task_determiner.srv import NewTask, NewTaskResponse

from std_srvs.srv import Empty, EmptyResponse


# POINTS
p1 = [0.04327964782714844, 0.3255343437194824, 0.0, 0.0, 0.0, 0.0, 1.0]
p2 = [0.6958894729614258, 0.919379711151123, 0.0, 0.0, 0.0, 0.0, 1.0]
p3 = [1.122640609741211, 1.581540584564209, 0.0, 0.0, 0.0, 0.0, 1.0]
p4 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
p5 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

POS_LIST = [p1, p2, p3]


class FakeTaskGenerator:
    def __init__(self):
        self.pos_list_ = []

    def new_task(self, req):
        print("list of points: ", self.pos_list_)
        res = NewTaskResponse()
        if self.pos_list_ == []:
            res.next_step = "wait"
            res.if_measure_after_movement = False
            return res
        res.next_step = "move"
        res.goal_pose = wrap_vector(self.pos_list_.pop(0))
        res.if_measure_after_movement = True
        return res

    def load_points(self, req):
        print("Loading new points")
        res = EmptyResponse()
        self.pos_list_ = POS_LIST.copy()
        return res

# wraps pose vector into PoseStamped
def wrap_vector(vec):
    pose_ = PoseStamped()
    pose_.header.stamp = rospy.Time.now()
    pose_.header.frame_id = 'map'

    pose_.pose.position.x = vec[0]
    pose_.pose.position.y = vec[1]
    pose_.pose.position.z = vec[2]
    pose_.pose.orientation.x = vec[3]
    pose_.pose.orientation.y = vec[4]
    pose_.pose.orientation.z = vec[5]
    pose_.pose.orientation.w = vec[6]
    return pose_

def main():
    rospy.init_node('fake_task_generator_node')

    fake_task_generator = FakeTaskGenerator()
    s = rospy.Service('fake_task_generator_node/new_task', NewTask, fake_task_generator.new_task)
    init = rospy.Service('fake_task_generator_init', Empty, fake_task_generator.load_points)

    rospy.spin()

if __name__ == '__main__':
    main()
