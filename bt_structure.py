import py_trees
import py_trees_ros
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, atan2
import sys


class Check_Obstacle(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Check_Obstacle, self).__init__(name)
        self.distance_to_obstacle = 0
        self.laser_scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)



    def laser_scan_callback(self, msg):
    # Assuming the target distance is at a specific angle (e.g., straight ahead)
        target_angle_index = 0 # len(msg.ranges) // 2
        self.distance_to_obstacle = msg.ranges[target_angle_index]

    def update(self):
        rospy.loginfo("Performing Obstacle_Check")
        rospy.loginfo("Distance to obstacle: {:.2f} meters".format(self.distance_to_obstacle))
        if self.distance_to_obstacle != 0:
           rospy.loginfo("Distance to obstacle: {:.2f} meters".format(self.distance_to_obstacle))
        if self.distance_to_obstacle < 1.2:
            success_condition_met = 1
        else:
            success_condition_met = 0
            rospy.loginfo("Distance > 1.2 ")

        if success_condition_met:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class Spin_Robot(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Spin_Robot, self).__init__(name)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_msg = Twist()

    def update(self):
        rospy.loginfo("Performing Spin_Robot")

        self.twist_msg.angular.z = 0.2

        self.cmd_vel_publisher.publish(self.twist_msg)

        return py_trees.common.Status.SUCCESS



class Move_to_Goal(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Move_to_Goal, self).__init__(name)
        self.goal_position = None
        self.success_flag = 0
        self.goal_subscriber = rospy.Subscriber('/goal_position', PoseStamped, self.goal_callback)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self.goal_reached_threshold = 0.2


    def goal_callback(self, msg):
        # Extract the goal position from the message
        self.goal_position = msg.pose.position

    def pose_callback(self, msg):
        # Extract the robot's current position from the message
        self.current_position = msg.pose.pose.position

    def move_to_goal(self):
        if self.goal_position is not None and self.current_position is not None:
            
            distance_to_goal = sqrt((self.goal_position.x - self.current_position.x) ** 2 +
                                    (self.goal_position.y - self.current_position.y) ** 2)

            rospy.loginfo("Distance to goal: {:.2f}".format(distance_to_goal))

            # Check if the robot has reached the goal based on the threshold
            if distance_to_goal <= self.goal_reached_threshold:
                rospy.loginfo("Goal reached!")
                linear_velocity = 0
                angular_velocity = 0.0 
                self.success_flag = 1
                return py_trees.common.Status.SUCCESS
            else:
                rospy.loginfo("mOVE TO GOAL RUNNING")
                linear_velocity = 0.1 
                angular_velocity = 0.00  
                self.success_flag = 0
                twist_msg = Twist()
                twist_msg.linear.x = linear_velocity
                twist_msg.angular.z = angular_velocity
                self.cmd_vel_publisher.publish(twist_msg)


                return py_trees.common.Status.RUNNING



    def update(self):
        rospy.loginfo("Performing Move_to_Goal")      
        self.move_to_goal()
        rospy.loginfo("Move to update function")
        if self.success_flag:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING



class Rot_to_Goal(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Rot_to_Goal, self).__init__(name)
        self.goal_position = None
        self.robot_orientation = None 
        self.robot_position = None
        self.robot_theta = 0
        self.success_flag = 0
        self.orientation_subscriber = rospy.Subscriber('/odom', Odometry, self.orientation_callback)
        self.goal_subscriber = rospy.Subscriber('/goal_position', PoseStamped, self.goal_callback)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def goal_callback(self, msg):
        # Extract the goal orientation (quaternion) from the message
        self.goal_position = msg.pose.position

        # Convert quaternion to Euler angles (roll, pitch, yaw)
       # (roll, pitch, self.goal_theta) = euler_from_quaternion([self.goal_orientation.x, self.goal_orientation.y, self.goal_orientation.z, self.goal_orientation.w])

        # Rotate the robot towards the goal (in this example, we'll only rotate in the z-axis)
      #  angular_velocity = 0.05
      #  if self.goal_theta > 0:
      #      angular_velocity *= -1  # Rotate clockwise
      #  self.rotate_robot(angular_velocity)


    def rotate_robot(self, angular_velocity):
        twist_msg = Twist()
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist_msg)
        rospy.loginfo("Rotate_Robot")

    def orientation_callback(self, msg):
        # Store the robot's orientation
        self.robot_orientation = msg.pose.pose.orientation
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        (roll, pitch, self.robot_theta) = euler_from_quaternion([self.robot_orientation.x, self.robot_orientation.y, self.robot_orientation.z, self.robot_orientation.w])
        
        self.robot_position = msg.pose.pose.position

    def turn_to_goal(self):
        angular_velocity = 0.09
        error = 0.04
#        rospy.loginfo(self.robot_position)
        if self.goal_position is not None and self.robot_position is not None:
            angle_toGoal = atan2(self.goal_position.y - self.robot_position.y, self.goal_position.x - self.robot_position.x)
            rospy.loginfo("Diff in Goal ")
  #          rospy.loginfo(angle_toGoal)
            if abs(angle_toGoal - self.robot_theta) > error:
                self.rotate_robot(angular_velocity)
                self.success_flag = 0
                rospy.loginfo(angle_toGoal - self.robot_theta)
                return py_trees.common.Status.RUNNING
            else:  
                self.rotate_robot(0)
                self.success_flag = 1
                rospy.loginfo("Rotation Success ")
                return py_trees.common.Status.SUCCESS
        else:
            rospy.loginfo("Something is None")


    def update(self):
        rospy.loginfo("Performing Rot_to_Goal")
        self.turn_to_goal()
        rospy.loginfo("OUT of Turn_to_goal")
        if self.success_flag:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


class Fallback(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Fallback, self).__init__(name)

    def update(self):
        rospy.loginfo("Performing Fallback")
        return py_trees.common.Status.SUCCESS

def create_behavior_tree():
    root = py_trees.composites.Selector("Root Selector")
    left1 = py_trees.composites.Sequence("Left1 Sequence")
    right1 = py_trees.composites.Sequence("Right1 Sequence")
    check_obstacle = Check_Obstacle("Check Obstacle")
    spin_robot = Spin_Robot("Spin Robot")
    move_to_goal = Move_to_Goal("Move to Goal")
    rot_to_goal = Rot_to_Goal("Rotate to Goal")
    fallback = Fallback("Fallback")
    left1.add_children([check_obstacle,spin_robot])
    right1.add_children([rot_to_goal,move_to_goal])
    root.add_children([left1,right1,fallback])
    return root

if __name__ == "__main__":
    try:
        rospy.init_node("simple_behavior_tree")
       # publish_goal()
        behavior_tree = create_behavior_tree()
        tree = py_trees_ros.trees.BehaviourTree(behavior_tree)
        tree.setup(timeout=15)
        tree.tick_tock(
            sleep_ms=10,
            number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK
        )
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS shutdown request received.")
        sys.exit(0)





































