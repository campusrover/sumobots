import rospy
from geometry_msgs.msg import Twist

rospy.init_node('test')

pub = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=1)

t = Twist()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    t.linear.x = 1.8
    print(t.linear.x)
    pub.publish(t)