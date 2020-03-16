#!/usr/bin/env python
import rospy
import math
import tf
import moveit_commander
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, MoveGroupAction
from moveit_msgs.srv import GetPlanningScene
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
import actionlib

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()
    ac = actionlib.SimpleActionClient(
        '/ariac/gantry/move_group', MoveGroupAction)
    ac.wait_for_server()

    group_name = "Full_Robot"
    robot_description = "robot_description"

    moveit_commander.MoveGroupCommander(
        "Full_Robot",
        robot_description="/ariac/gantry/robot_description",
        ns="/ariac/gantry")
    scene_interface = moveit_commander.PlanningSceneInterface("/ariac/gantry")
    # get_planning_scene
    # try:
    #     rospy.wait_for_service('/get_planning_scene', timeout=20)
    #     get_planning_scene = rospy.ServiceProxy(
    #         "/ariac/gantry/get_planning_scene", GetPlanningScene)
    # except BaseException:
    #     get_planning_scene = None

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            for i in range(1, 12):
                object_name = "shelf" + str(i)
                (trans, rot) = listener.lookupTransform(
                    '/world', "/" + object_name + "_frame", rospy.Time(0))

                rospy.loginfo(trans)
                rospy.loginfo(rot)

                box_pose = geometry_msgs.msg.PoseStamped()
                box_pose.header.frame_id = "world"
                box_pose.pose.position.x = trans[0] - 2
                box_pose.pose.position.y = trans[1] - 0.63
                box_pose.pose.position.z = trans[2]
                box_pose.pose.orientation.x = rot[0]
                box_pose.pose.orientation.y = rot[1]
                box_pose.pose.orientation.z = rot[2]
                box_pose.pose.orientation.w = rot[3]

                scene_interface.add_box(
                    object_name, box_pose, (4.0, 1.26, 3.0))
                msg = object_name + "is added!!"
                rospy.loginfo(msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

            rate.sleep()
