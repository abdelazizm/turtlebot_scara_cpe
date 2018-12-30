#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script
import roslib

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()


	# Customize the following values so they are appropriate for your location

	position = {'x': 3.52, 'y' : -1.7}
	quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
	rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
	success = navigator.goto(position, quaternion)

	if success:
		rospy.loginfo("Hooray, reached the coin position")
	else:
		rospy.loginfo("The base failed to reach the desired pose")

	# Sleep to give the last log messages time to be sent
	rospy.sleep(1)


	position_coin1 = {'x': 2.589333, 'y' : -0.321235}
	position_coin2 = {'x': 2.556730, 'y' : -0.100126}
	position_final1= {'x': 2.538429, 'y' : 0.714428}
	position_final2= {'x': 2.505826, 'y' : 0.935537}
	quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

	rospy.loginfo("Go to (%s, %s) pose", position_coin1['x'], position_coin1['y'])
	success = navigator.goto(position_coin1, quaternion)

	if success:
		rospy.loginfo("Hooray, reached the coin position")
	else:
		rospy.loginfo("The base failed to reach the desired pose")

	# Sleep to give the last log messages time to be sent
	rospy.sleep(1)
	rospy.loginfo("Go to (%s, %s) pose", position_final1['x'], position_final1['y'])
	success = navigator.goto(position_final1, quaternion)
	if success:
		rospy.loginfo("Hooray, reached the final position")
	else:
		rospy.loginfo("The base failed to reach the desired pose")
	rospy.sleep(1)
	rospy.loginfo("Go to (%s, %s) pose", position_coin2['x'], position_coin2['y'])
	success = navigator.goto(position_coin2, quaternion)

	if success:
		rospy.loginfo("Hooray, reached the coin position")
	else:
		rospy.loginfo("The base failed to reach the desired pose")

	# Sleep to give the last log messages time to be sent
	rospy.sleep(1)
	rospy.loginfo("Go to (%s, %s) pose", position_final2['x'], position_final2['y'])
	success = navigator.goto(position_final2, quaternion)
	if success:
		rospy.loginfo("Hooray, reached the final position")
	else:
		rospy.loginfo("The base failed to reach the desired pose")
	rospy.sleep(1)


    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

