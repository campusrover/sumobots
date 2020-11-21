#!/usr/bin/env python  
import rospy

import tf_conversions
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

# Called each time the robot publishes a new pose. We inform tf2 that there's a new
# transform between this robot and the world coordinate system by creating a TransformBroadcaster
# and calling sendTransform.
def handle_robot_pose(msg, robot_name):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

# Fill in the Transform with the current time, the from and to frame_ids and
# the 7 components of the transform, three for translation and 4 (!) for rotation.
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = robot_name
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

# Publish the trandsform.
    br.sendTransform(t)


if __name__ == '__main__':
# Create a node, grab a parameter indicating the name of the robot, and subscribe that
# robot's pose, /robotX/pose. By using a parameter we can launch this node and supply 
# the robot name later.
    rospy.init_node('tf2_broadcaster')
    robot_name = rospy.get_param('~robot', 'robot1')
    rospy.Subscriber('/%s/pose' % robot_name, Odometry.pose.pose, handle_robot_pose, robot_name)
    rospy.spin()