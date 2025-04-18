#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Pose2D
from tf.transformations import quaternion_from_euler
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MovementManager:
    def __init__(self):
        self.rate = rospy.Rate(5)
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Ожидание действия move_base")
        self.client.wait_for_server()
        rospy.loginfo("Действие move_base доступно")
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Подписка на топик /odom_pose2d для получения текущей ориентации
        self.odom_pose = None
        self.odom_sub = rospy.Subscriber('/odom_pose2d', Pose2D, self.odom_callback)

    def odom_callback(self, msg):
        """Обработка данных из топика /odom_pose2d."""
        self.odom_pose = msg

    def send_goal(self, point):
        """Отправка цели в move_base."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(point['x'])
        goal.target_pose.pose.position.y = float(point['y'])
        q = quaternion_from_euler(0, 0, math.radians(float(point['theta'])))
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        rospy.loginfo(f"Отправка цели: {point}")
        self.client.send_goal(goal)

    def rotate_to_angle(self, target_angle_degrees):
        """Поворот робота на целевой угол."""
        if self.odom_pose is None:
            rospy.logwarn("Нет данных об ориентации. Пропуск поворота.")
            return

        target_angle_radians = math.radians(target_angle_degrees)
        current_theta = self.odom_pose.theta
        target_theta = normalize_angle(current_theta + target_angle_radians)

        twist = Twist()
        angular_speed = 0.5  # Радианы в секунду

        while not rospy.is_shutdown():
            current_theta = self.odom_pose.theta
            angle_diff = normalize_angle(target_theta - current_theta)

            if abs(angle_diff) < 0.05:  # Допустимая погрешность (в радианах)
                break

            twist.angular.z = angular_speed if angle_diff > 0 else -angular_speed
            self.cmd_pub.publish(twist)
            self.rate.sleep()

        twist.angular.z = 0
        self.cmd_pub.publish(twist)  # Остановка вращения
        rospy.loginfo(f"Поворот завершен до угла: {math.degrees(target_theta)} градусов")


def normalize_angle(angle):
    """Нормализация угла в диапазоне [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))