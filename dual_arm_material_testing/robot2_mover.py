#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sqrt


# moves robot2 10 mm in z direction and backs to start position

class Robot2Mover():

    def __init__(self):



        self.twist_pub = rospy.Publisher('/mur620b/UR10_r/twist_controller/command_collision_free', Twist, queue_size=10)

        # Subscriber für Pose
        rospy.Subscriber('/mur620b/UR10_r/ur_calibrated_pose', PoseStamped, self.pose_callback)
        self.current_pose = None
        self.current_pose = None
        self.Kp, self.Ki, self.Kd = 0.8, 0.08, 0.1
        self.target_position = [0.5996488732821253, 0.13827128899619845, 0.3676819607615602+0.015]  # Beispiel-Zielposition (x, y, z) in Metern
        
        self.integral = 0
        self.prev_error = 0
        self.rate = rospy.Rate(100)  # 10 Hz
        self.dt = 1/100.0  # Zeitintervall basierend auf der Rate


    def execute_combined_motion_no_return(self, move_x_mm=0.0, 
                                                t1=2.0, 
                                                t2=2.0):
        """
        Führt kombinierte Bewegung aus (Translation + Rotation), wartet t2 Sekunden – 
        KEINE Rückfahrt zur Startposition.
        """
        if t1 <= 0.0:
            rospy.logwarn("t1 muss > 0 sein")
            return

        rospy.loginfo("Starte kombinierte Bewegung (ohne Rückfahrt)...")

        rate = rospy.Rate(100)

        # Umrechnung Einheiten grad zu rad, mm zu m
        move_x_m = move_x_mm / 1000.0

        # Berechnung der benötigten Geschwindigkeiten
        linear_x_velocity = move_x_m / t1

        twist = Twist()
        twist.linear.x = linear_x_velocity

        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_time
            if elapsed >= t1:
                break
            self.twist_pub.publish(twist)
            rate.sleep()

        self.publish_zero_twist()

        rospy.loginfo(f"Pausiere nach Bewegung für {t2} Sekunden...")
        rospy.sleep(t2)


    def distance_to_target(self):
        if self.current_pose is None:
            return float('inf')

        dx = self.target_position[0] - self.current_pose.position.x
        dy = self.target_position[1] - self.current_pose.position.y
        dz = self.target_position[2] - self.current_pose.position.z
        return sqrt(dx**2 + dy**2 + dz**2)
        #return abs(dz)

    def move_to_start_fast(self, tolerance=0.0005):

        self.integral = 0
        self.prev_error = 0

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.loginfo("Warte auf aktuelle Pose...")
                self.rate.sleep()
                continue

            distance = self.distance_to_target()
            print(f"Aktueller Abstand zur Zielposition: {distance:.6f} m")

            if distance < tolerance:
                rospy.loginfo("Roboter ist in der Startposition.")
                self.publish_zero_twist()
                rospy.sleep(1.0)
                break

            # Steuerung per Proportionalregelung
            twist = Twist()
            #twist.linear.x = 0.5 * (self.target_position[0] - self.current_pose.position.x)
            twist.linear.x = self.update(self.target_position[0] - self.current_pose.position.x)
            twist.linear.y = 0.5 * (self.target_position[1] - self.current_pose.position.y)
            twist.linear.z = 0.5 * (self.target_position[2] - self.current_pose.position.z) 

            # Begrenzung der Geschwindigkeit
            max_vel = 0.2  # m/s
            twist.linear.x = max(-max_vel, min(twist.linear.x, max_vel))
            twist.linear.y = max(-max_vel, min(twist.linear.y, max_vel))
            twist.linear.z = max(-max_vel, min(twist.linear.z, max_vel))

            self.twist_pub.publish(twist)
            self.rate.sleep()

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        #print(self.Kp*error + self.Ki*self.integral + self.Kd*derivative)
        return self.Kp*error + self.Ki*self.integral + self.Kd*derivative

    def publish_zero_twist(self):
        stop_twist = Twist()
        self.twist_pub.publish(stop_twist)

    def pose_callback(self, msg):
        self.current_pose = msg.pose


if __name__ == "__main__":
    rospy.init_node("robot2_mover")

    mover = Robot2Mover()
    rospy.sleep(0.1)  # Warte auf Initialisierung
    mover.move_to_start_fast()

    for i in range(0,10):
        if rospy.is_shutdown():
            break
        rospy.sleep(1.0)  # Warte auf Initialisierung
        mover.move_to_start_fast()
        mover.execute_combined_motion_no_return(
            move_x_mm=i * 4.0, 
            t1=4.0, 
            t2=2.0
        )

    mover.move_to_start_fast()
    # mover.execute_combined_motion_no_return(
    #         move_x_mm=10.0, 
    #         t1=4.0, 
    #         t2=2.0
    #     )
