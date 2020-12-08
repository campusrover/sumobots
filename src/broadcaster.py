#!/usr/bin/env python  
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

'''
---------------------------------------------------------------------------------------------------
Broadcaster node for robot coordinate transforms.
Modified from ROS wiki TF2 tutorial:

-------- http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29 --------
---------------------------------------------------------------------------------------------------
'''

# called each time the robot publishes a new pose. We inform tf2 there's a new transform between
# this robot and the world coordinate system by creating a TransformBroadcaster and calling
# sendTransform.
def handle_robot_pose(msg, robot_name):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # Fill in the Transform with the current time, the from and to frame_ids and
    # the 7 components of the transform, three for translation and four for rotation.
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = robot_name
    msg = msg.pose.pose
    t.transform.translation.x = msg.position.x
    t.transform.translation.y = msg.position.y
    t.transform.translation.z = msg.position.z
    t.transform.rotation.x = msg.orientation.x
    t.transform.rotation.y = msg.orientation.y
    t.transform.rotation.z = msg.orientation.z
    t.transform.rotation.w = msg.orientation.w

    # Publish the trandsform.
    br.sendTransform(t)


if __name__ == '__main__':
    # Create a node, grab a parameter indicating the name of the robot, and subscribe to that
    # robot's pose, /robotX/odom. By using a parameter we can launch this node and supply 
    # the robot name later.
    rospy.init_node('tf2_broadcaster')
    robot_name = rospy.get_param('~robot', 'robot1')
    rospy.Subscriber('/%s/odom' % robot_name, Odometry, handle_robot_pose, robot_name)
    rospy.spin()