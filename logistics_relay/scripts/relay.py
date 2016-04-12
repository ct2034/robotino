#!/usr/bin/env python2

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point


def pubGoal(x, y, th):
    ps = PoseStamped()
    ps.header.seq = 1
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "map"
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    ps.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, th)

    pub.publish(ps)


def callbackGoal(data):
    rospy.loginfo(
        "RELAY (" +
        str(data.x) +
        "; " +
        str(data.y) +
        "; " +
        str(data.z) +
        ")"
    )
    pubGoal(data.x, data.y, data.z)


if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)

        rospy.Subscriber("logistics_goal", Point, callbackGoal)
        pub = rospy.Publisher(
            'move_base_simple/goal', PoseStamped, queue_size=10)

        rospy.spin()

        ros except rospy.ROSInterruptException:
        pass
