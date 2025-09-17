#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from math import sqrt

class RobotMover:
    def __init__(self):
        rospy.init_node('robot_mover', anonymous=True)

        # Zielposition (Startposition des Roboters)
        self.target_position = rospy.get_param('~start_position', [-0.8399000197524626, 0.30141801515874383, 0.6408312906331164])  # z.B. [x, y, z]

        # Publisher für Twist
        self.twist_pub = rospy.Publisher('/mur620a/UR10_r/twist_controller/controller_input', Twist, queue_size=10)

        # Subscriber für Pose
        rospy.Subscriber('/mur620a/UR10_r/ur_calibrated_pose', PoseStamped, self.pose_callback)

        self.current_pose = None
        self.rate = rospy.Rate(10)  # 10 Hz

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def distance_to_target(self):
        if self.current_pose is None:
            return float('inf')


        print(self.target_position)
        print(self.current_pose.position)
        dx = self.target_position[0] - self.current_pose.position.x
        dy = self.target_position[1] - self.current_pose.position.y
        dz = self.target_position[2] - self.current_pose.position.z
        return sqrt(dx**2 + dy**2 + dz**2)

    def move_to_start(self, tolerance=0.01):
        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.loginfo("Warte auf aktuelle Pose...")
                self.rate.sleep()
                continue

            distance = self.distance_to_target()
            rospy.loginfo(f"Aktueller Abstand zur Zielposition: {distance:.4f} m")

            if distance < tolerance:
                rospy.loginfo("Roboter ist in der Startposition.")
                self.publish_zero_twist()
                break

            # Steuerung per Proportionalregelung
            twist = Twist()
            twist.linear.x = 0.1 * (self.target_position[0] - self.current_pose.position.x)
            twist.linear.y = 0.1 * (self.target_position[1] - self.current_pose.position.y)
            twist.linear.z = 0.1 * (self.target_position[2] - self.current_pose.position.z)

            # Begrenzung der Geschwindigkeit
            max_vel = 0.1  # m/s
            twist.linear.x = max(-max_vel, min(twist.linear.x, max_vel))
            twist.linear.y = max(-max_vel, min(twist.linear.y, max_vel))
            twist.linear.z = max(-max_vel, min(twist.linear.z, max_vel))

            self.twist_pub.publish(twist)
            self.rate.sleep()

    def publish_zero_twist(self):
        stop_twist = Twist()
        self.twist_pub.publish(stop_twist)


if __name__ == '__main__':
    try:
        mover = RobotMover()
        mover.move_to_start()
    except rospy.ROSInterruptException:
        pass
