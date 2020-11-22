import rospy
from geometry_msgs.msg import Twist

rospy.init_node('test')

pub1 = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('robot2/cmd_vel', Twist, queue_size=1)

t1 = Twist()
t2 = Twist()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if ((rospy.Time.now().to_sec()* 5 // 1) % 2 == 0):
        t1.linear.x = 2
        t2.linear.x = 2 

    else:
        t1.linear.x = -2
        t2.linear.x = 2
    #pub1.publish(t1)
    pub2.publish(t2)