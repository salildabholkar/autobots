#!/usr/bin/env python

# Original Code: https://github.com/markwsilliman/turtlebot/blob/master/goforward_and_avoid_obstacle.py
# TurtleBot must have minimal.launch & amcl_demo.launch running prior to starting this script.

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from objects import all_objects, bins
import numpy as np
import math


class Move():
    def __init__(self):
        rospy.init_node('move_bot')

        self.reached = False
        self.bin = ''

        # tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")

        # allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))
        self.picked = False

        self.all_objects = all_objects

        # Robot pose
        self.robot_pose = Point(0, 0, 0)
        positions = [x[1] for x in self.all_objects]
        self.distances = [math.sqrt((point.x - self.robot_pose.x) ** 2 + (point.y - self.robot_pose.y) ** 2 + (
                point.z - self.robot_pose.z) ** 2) for point in positions]

        # what to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("init move node")
        self.update_pose()

    # For deleting models from the environment
    @staticmethod
    def del_model(model_name):
        del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)  # Handle to model spawner
        rospy.wait_for_service('gazebo/delete_model')  # Wait for the model loader to be ready
        del_model_prox(model_name)  # Remove from Gazebo

    def update_pose(self):
        self.robot_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose.position

        nearest, dist, elems = self.get_nearest_object_and_index()

        name = self.all_objects[nearest][0]
        id = self.all_objects[nearest][2]

        if not name.startswith('pickable'):
            self.all_objects.pop(nearest)
            self.reached = False

        elif self.picked:
            rospy.loginfo('Dropping to: ' + self.bin)
            self.goto(bins[self.bin])
            self.picked = False

        elif dist < 1:
            rospy.loginfo('Picked up: ' + name)
            self.picked = True
            self.bin = self.all_objects[nearest][3]
            self.all_objects.pop(nearest)
            self.del_model(id)
            if elems == 1:
                rospy.loginfo('done')
                rospy.signal_shutdown('done')

        else:
            rospy.loginfo('Going to pick: ' + name + ' ' + str(dist))
            self.goto(self.all_objects[nearest][1])


    def get_nearest_object_and_index(self):
        positions = [x[1] for x in self.all_objects]
        dist = [math.sqrt((point.x - self.robot_pose.x) ** 2 + (point.y - self.robot_pose.y) ** 2 + (point.z - self.robot_pose.z) ** 2) for point in positions]
        print dist
        mini = np.argmin(dist)
        return mini, dist[mini], len(dist)

    def goto(self, goal_pose):
        self.reached = True
        goal = MoveBaseGoal()
        goal.target_pose.pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = goal_pose

        # Start moving
        print 'sent goal signal'
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 200 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(30))

        state = self.move_base.get_state()

        if success or state == GoalStatus.SUCCEEDED:
            # We made it!
            rospy.loginfo("--------Reached, continue to next----------")
            self.reached = False
        else:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach after 200s")

    def shutdown(self):
        rospy.loginfo("Stop")
        self.move_base.cancel_goal()

    def run(self):
        r = rospy.Rate(100)  # 100hz
        while not rospy.is_shutdown():
            if not self.reached:
                self.update_pose()
            r.sleep()


if __name__ == '__main__':
    try:
        Move().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
