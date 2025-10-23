#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from math import sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import math
import subprocess
import os
import signal
from datetime import datetime
import tf.transformations as tft


class RobotMover:
    def __init__(self):
        rospy.init_node('robot_mover', anonymous=True)

        # Zielposition (Startposition des Roboters)
        #self.target_position = rospy.get_param('~start_pos', [0.33263377919966164, -0.028678860128717395, -0.028678860128717395])  # z.B. [x, y, z]
        self.target_position = rospy.get_param('~start_pos', [-0.8320953268242405, 0.3489168508730299, 0.497+0.00])  # z.B. [x, y, z]
        self.target_orientation = rospy.get_param('~start_orientation', [1.0, 0.0, 0.0, 0.0])  # Quaternion [x, y, z, w]

        # Publisher für Twist
        self.twist_pub = rospy.Publisher('/mur620a/UR10_r/twist_controller/command_collision_free', Twist, queue_size=10)

        # Subscriber für Pose
        rospy.Subscriber('/mur620a/UR10_r/ur_calibrated_pose', PoseStamped, self.pose_callback)

        self.current_pose = None
        self.Kp, self.Ki, self.Kd = 0.8, 0.08, 0.1
        
        self.integral = 0
        self.prev_error = 0
        self.rate = rospy.Rate(100)  # 10 Hz
        self.dt = 1/100.0  # Zeitintervall basierend auf der Rate

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def distance_to_target(self):
        if self.current_pose is None:
            return float('inf')

        dx = self.target_position[0] - self.current_pose.position.x
        dy = self.target_position[1] - self.current_pose.position.y
        dz = self.target_position[2] - self.current_pose.position.z
        #return sqrt(dx**2 + dy**2 + dz**2)
        return abs(dz)

    def move_to_start_fast(self, tolerance=0.0002):

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

    def reset_orientation(self):

        self.integral = 0
        self.prev_error = 0

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.loginfo("Warte auf aktuelle Pose...")
                self.rate.sleep()
                continue

            # Orientierung korrigieren 
            current_orientation_euler = euler_from_quaternion([
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ])
            target_orientation_euler = euler_from_quaternion(self.target_orientation)   
            orientation_error = [target_orientation_euler[i] - current_orientation_euler[i] for i in range(3)]

            for i in range(3):
                orientation_error[i] = math.atan2(math.sin(orientation_error[i]), math.cos(orientation_error[i]))

            if all(abs(err) < 0.01 for err in orientation_error):
                rospy.loginfo("Roboter hat die Zielorientierung erreicht.")
                self.publish_zero_twist()
                break


            twist = Twist()
            twist.angular.x = 0.5 * orientation_error[0]
            twist.angular.y = 0.5 * orientation_error[1]
            #twist.angular.z = 0.5 * orientation_error[2] 
            twist.angular.z = self.update(orientation_error[2])

            max_ang_vel = 0.3  # rad/s
            twist.angular.x = max(-max_ang_vel, min(twist.angular.x, max_ang_vel))
            twist.angular.y = max(-max_ang_vel, min(twist.angular.y, max_ang_vel))
            twist.angular.z = max(-max_ang_vel, min(twist.angular.z, max_ang_vel))

            # angular velocity has to be negative for clockwise rotation
            if twist.angular.z > 0.1:
                twist.angular.z = -0.1
                rospy.logwarn_throttle(1, "Setze Drehgeschwindigkeit auf -0.05 rad/s für Uhrzeigersinn")

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

    def execute_combined_motion_no_return(self, move_z_mm=0.0, 
                                                R_x_deg=0.0, 
                                                R_y_deg=0.0, 
                                                R_z_deg=0.0, 
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
        move_z_m = move_z_mm / 1000.0
        R_x_rad = math.radians(R_x_deg)
        R_y_rad = math.radians(R_y_deg)
        R_z_rad = math.radians(R_z_deg)

        # Berechnung der benötigten Geschwindigkeiten
        linear_z_velocity = move_z_m / t1
        angular_x_velocity = R_x_rad / t1
        angular_y_velocity = R_y_rad / t1
        angular_z_velocity = R_z_rad / t1



        twist = Twist()
        twist.linear.z = linear_z_velocity
        twist.angular.x = angular_x_velocity
        twist.angular.y = angular_y_velocity
        twist.angular.z = angular_z_velocity

        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_time
            if elapsed >= t1:
                break
            self.twist_pub.publish(twist)
            rate.sleep()

        self.publish_zero_twist()

        rospy.loginfo(f"Pausiere nach Bewegung für {t2} Sekunden...")
        # rospy.sleep(t2)
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_time
            if elapsed >= t2:
                break
            rate.sleep()

    def record_measurement(self, move_z_mm=0.0, R_x_deg=0.0, R_y_deg=0.0, R_z_deg=0.0, t1=2.0, t2=2.0):
        """
        Führt eine kombinierte Bewegung aus, zeichnet relevante Topics auf,
        speichert rosbag mit passenden Parametern im Dateinamen.
        """

        # Erzeuge Dateiname
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"meas_z{int(move_z_mm)}_Rx{int(R_x_deg)}_Ry{int(R_y_deg)}_Rz{int(R_z_deg)}_t1-{t1}_t2-{t2}_{timestamp}.bag"
        filepath = os.path.join(os.path.expanduser("~"), "rosbags", filename)

        # Sicherstellen, dass Zielordner existiert
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        # rosbag Kommando
        topics = [
            "/mur620a/UR10_l/wrench",
            "/mur620a/UR10_r/wrench",
            "/mur620b/UR10_r/wrench",
            "/mur620a/UR10_l/ur_calibrated_pose",
            "/mur620a/UR10_r/ur_calibrated_pose",
            "/mur620b/UR10_r/ur_calibrated_pose"
        ]
        cmd = ["rosbag", "record", "-O", filepath] + topics

        rospy.loginfo(f"Starte rosbag Aufzeichnung: {filename}")
        rosbag_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setpgrp)

        # Kurze Wartezeit, damit rosbag startet (optional)
        rospy.sleep(1.0)

        # Bewegung ausführen (ohne Rückkehr)
        self.execute_combined_motion_no_return(
            move_z_mm=move_z_mm, 
            R_x_deg=R_x_deg, 
            R_y_deg=R_y_deg, 
            R_z_deg=R_z_deg, 
            t1=t1, 
            t2=t2
        )

        # rosbag Recording stoppen
        rospy.loginfo("Stoppe rosbag Aufzeichnung...")
        os.killpg(os.getpgid(rosbag_proc.pid), signal.SIGINT)
        rosbag_proc.wait()

        rospy.loginfo(f"Rosbag gespeichert unter: {filepath}")

        # Jetzt Rückfahrt
        rospy.loginfo("Fahre zurück zur Startposition...")
        self.reset_orientation()

        self.move_to_start_fast()
        

if __name__ == '__main__':
    try:
        #time.sleep(5)
        for i in range(0,1):
            if rospy.is_shutdown():
                break   
            rospy.loginfo(f"Durchlauf {i+1}/25")
            mover = RobotMover()
            mover.reset_orientation()
            mover.move_to_start_fast()

            # Beispielaufruf der Messung
            #mover.record_measurement(move_z_mm=i*1, R_x_deg=0.0, R_y_deg=0.0, R_z_deg=0, t1=2.0+i*0.2, t2=2.0)

            mover.record_measurement(move_z_mm=10.0, R_x_deg=0.0, R_y_deg=0.0, R_z_deg=0.0, t1=5.0, t2=240.0)

            #mover.record_measurement(move_z_mm=-5, R_x_deg=0.0, R_y_deg=0.0, R_z_deg=0.0, t1=5, t2=2.0)
            #mover.move_to_start()
    except rospy.ROSInterruptException:
        pass
