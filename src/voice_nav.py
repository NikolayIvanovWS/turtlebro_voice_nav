#!/usr/bin/env python3
import rospy
import math
import toml
import actionlib
from pathlib import Path
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist

class VoiceNav(object):
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)
        self.rate = rospy.Rate(5)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting move_base action")
        self.client.wait_for_server()
        rospy.loginfo(f"Have move_base action")
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.voice_sub = rospy.Subscriber('/robohead_controller/voice_recognizer_pocketsphinx/cmds_recognizer/commands', String, self.voice_command_cb)
        self.load_rooms_config()
        if 'стартоваяточка' in self.rooms_config['rooms']:
            self.start_point = self.rooms_config['rooms']['стартоваяточка']
        else:
            rospy.logerr("Стартовая точка не найдена в конфигурационном файле rooms.toml")
            rospy.signal_shutdown("Стартовая точка не найдена")
        self.current_point = self.start_point
        self.goal_sent = False  # Флаг для отслеживания отправки цели
        self.returning_home = False  # Флаг для отслеживания возвращения в стартовую точку
        self.is_inspecting = False  # Флаг для отслеживания выполнения команды "осмотрись"
        rospy.loginfo("Init done")

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def voice_command_cb(self, message):
        command = message.data.strip()
        rospy.loginfo(f"Received command: {command}")
        if self.is_movement_command(command):
            room_name = self.get_room_name_from_command(command)
            if room_name:
                rospy.loginfo(f"Moving to room: {room_name}")
                goal = self._goal_message_assemble(self.rooms_config['rooms'][room_name])
                self.client.send_goal(goal, done_cb=self.move_base_cb)
                self.goal_sent = True  # Устанавливаем флаг, что цель отправлена
                self.returning_home = False  # Сбрасываем флаг возвращения в стартовую точку
                self.is_inspecting = False  # Сбрасываем флаг осмотра
            else:
                rospy.loginfo("Room not found")
        elif self.is_inspect_command(command):
            if not self.is_inspecting:
                rospy.loginfo("Starting inspection sequence")
                self.is_inspecting = True
                self.perform_inspection()
        else:
            rospy.loginfo("Unknown command")

    def is_movement_command(self, command):
        prefix = "езжай в"
        return command.startswith(prefix)

    def get_room_name_from_command(self, command):
        prefix = "езжай в"
        if command.startswith(prefix):
            room_name = command[len(prefix):].strip()
            if room_name in self.rooms_config['rooms']:
                return room_name
        return None

    def is_inspect_command(self, command):
        return command.strip() == "осмотрись"

    def perform_inspection(self):
        angles = [-15, 30, -30, 15]
        for angle in angles:
            self.rotate(angle)
            rospy.sleep(1)  # Пауза между поворотами
        rospy.sleep(2)  # Пауза после завершения осмотра
        self.return_to_start()

    def rotate(self, angle):
        twist = Twist()
        angular_speed = 0.5  # Радианы в секунду
        target_angle = math.radians(angle)
        duration = abs(target_angle) / angular_speed
        twist.angular.z = angular_speed if target_angle > 0 else -angular_speed

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        twist.angular.z = 0
        self.cmd_pub.publish(twist)  # Остановка вращения
        rospy.loginfo(f"Rotated by {angle} degrees")

    def return_to_start(self):
        rospy.loginfo("Returning to start point after inspection")
        goal = self._goal_message_assemble(self.start_point)
        self.client.send_goal(goal, done_cb=self.return_home_cb)
        self.returning_home = True
        self.is_inspecting = False

    def move_base_cb(self, status, result):
        if self.goal_sent:  # Проверяем, была ли цель отправлена
            if status == GoalStatus.PREEMPTED:
                rospy.loginfo("Movement cancelled")
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo("Movement succeeded")
                if not self.returning_home:
                    rospy.loginfo("Robot has arrived at the destination and is waiting for a new command")
            self.goal_sent = False  # Сбрасываем флаг после обработки цели
        else:
            rospy.logwarn("Received comm state ACTIVE when in simple state DONE")

    def return_home_cb(self, status, result):
        if self.returning_home:  # Проверяем, возвращается ли робот в стартовую точку
            if status == GoalStatus.PREEMPTED:
                rospy.loginfo("Return home cancelled")
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo("Returned to start point")
            self.returning_home = False  # Сбрасываем флаг после возвращения в стартовую точку
        else:
            rospy.logwarn("Received comm state ACTIVE when in simple state DONE")

    def load_rooms_config(self):
        config_data_file = rospy.get_param('~rooms_config_file', str(
            Path(__file__).parent.absolute()) + '/../data/rooms.toml')
        rospy.loginfo(f"Loading rooms config file {config_data_file}")
        try:
            self.rooms_config = toml.load(config_data_file)
            rospy.loginfo(f"Rooms config loaded: {self.rooms_config}")
        except Exception as e:
            rospy.logerr(f"TOML parser failed: {e}")
            rospy.signal_shutdown("No valid TOML file")

    def _goal_message_assemble(self, point):
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
        rospy.loginfo("Created goal from point {} ".format(point))
        return goal

    def on_shutdown(self):
        rospy.loginfo("Shutdown VoiceNav")
        self.cmd_pub.publish(Twist())
        self.client.action_client.stop()
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        rospy.init_node('voice_nav_node')
        robot = VoiceNav()
        robot.spin()
    except rospy.ROSInterruptException:
        robot.on_shutdown()
        rospy.loginfo("VoiceNav stopped due to ROS interrupt")