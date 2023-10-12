#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal():
    rospy.init_node('goal_publisher')
    goal_pub = rospy.Publisher('/goal_position', PoseStamped, queue_size=10)
    rate = rospy.Rate(1)  # Publish goal every 1 second

    while not rospy.is_shutdown():
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = 'map'  # Set the frame ID appropriately
        goal_msg.pose.position.x = -2.0  # Set the goal x-coordinate
        goal_msg.pose.position.y = 4.0  # Set the goal y-coordinate
        goal_msg.pose.orientation.w = 1.0  # No rotation (quaternion)

        goal_pub.publish(goal_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
