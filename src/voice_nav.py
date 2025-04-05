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
from turtlebro_speech.srv import Speech, SpeechRequest

class VoiceNav(object):
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)
        self.rate = rospy.Rate(5)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting move_base action")
        self.client.wait_for_server()
        rospy.loginfo("Have move_base action")

        # Создаем издатель для управления скоростью
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Подписываемся на топик с голосовыми командами
        self.voice_sub = rospy.Subscriber('/robohead_controller/voice_recognizer_pocketsphinx/cmds_recognizer/commands', String, self.voice_command_cb)

        # Загружаем конфигурационные файлы
        self.load_points_config()
        self.load_commands_config()
        self.load_speech_config()

        # Инициализация клиента для сервиса озвучивания
        self.speech_service = rospy.ServiceProxy('festival_speech', Speech)
        rospy.loginfo("Waiting for festival_speech service")
        self.speech_service.wait_for_service()
        rospy.loginfo("Have festival_speech service")

        # Проверяем наличие стартовой точки
        if 'стартовуюточку' in self.points_config['points']:
            self.start_point = self.points_config['points']['стартовуюточку']
        else:
            rospy.logerr("Стартовая точка не найдена в конфигурационном файле points.toml")
            rospy.signal_shutdown("Стартовая точка не найдена")

        # Инициализация переменных
        self.current_point = None  # Текущая точка
        self.goal_sent = False  # Флаг отправки цели
        self.returning_home = False  # Флаг возвращения в стартовую точку
        self.is_inspecting = False  # Флаг выполнения осмотра
        rospy.loginfo("Init done")

    def load_points_config(self):
        """Загрузка конфигурации точек."""
        config_data_file = rospy.get_param('~points_config_file', str(
            Path(__file__).parent.absolute()) + '/../data/points.toml')
        rospy.loginfo(f"Loading points config file {config_data_file}")
        try:
            self.points_config = toml.load(config_data_file)
            rospy.loginfo(f"Points config loaded: {self.points_config}")
        except Exception as e:
            rospy.logerr(f"TOML parser failed: {e}")
            rospy.signal_shutdown("No valid TOML file")

    def load_commands_config(self):
        """Загрузка конфигурации ключевых фраз."""
        config_data_file = rospy.get_param('~commands_config_file', str(
            Path(__file__).parent.absolute()) + '/../data/commands.toml')
        rospy.loginfo(f"Loading commands config file {config_data_file}")
        try:
            self.commands_config = toml.load(config_data_file)
            rospy.loginfo(f"Commands config loaded: {self.commands_config}")
        except Exception as e:
            rospy.logerr(f"TOML parser failed: {e}")
            rospy.signal_shutdown("No valid TOML file")

    def load_speech_config(self):
        """Загрузка конфигурации напоминаний."""
        config_data_file = rospy.get_param('~speech_config_file', str(
            Path(__file__).parent.absolute()) + '/../data/speech.toml')
        rospy.loginfo(f"Loading speech config file {config_data_file}")
        try:
            self.speech_config = toml.load(config_data_file)
            rospy.loginfo(f"Speech config loaded: {self.speech_config}")
        except Exception as e:
            rospy.logerr(f"TOML parser failed: {e}")
            rospy.signal_shutdown("No valid TOML file")

    def voice_command_cb(self, message):
        """Обработка голосовых команд."""
        if self.returning_home:
            rospy.loginfo("Robot is returning home, ignoring commands")
            return

        command = message.data.strip()
        rospy.loginfo(f"Received command: {command}")

        # Проверка команды на соответствие ключевым фразам для перемещения
        for phrase in self.commands_config['movement_phrases']['phrases']:
            if command.startswith(phrase):
                point_name = self.get_point_name_from_command(command, phrase)
                if point_name:
                    rospy.loginfo(f"Moving to point: {point_name}")
                    self.current_point = point_name
                    goal = self._goal_message_assemble(self.points_config['points'][point_name])
                    self.client.send_goal(goal, done_cb=self.move_base_cb)
                    self.goal_sent = True
                    return

        # Проверка команды "осмотрись"
        if self.is_inspect_command(command):
            if not self.is_inspecting:
                rospy.loginfo("Starting inspection sequence")
                self.is_inspecting = True
                self.perform_inspection()
            return

        # Проверка команды "напомни"
        if command.startswith("напомни"):
            reminder_key = command[len("напомни"):].strip()
            if reminder_key in self.speech_config['reminders']:
                reminder_text = self.speech_config['reminders'][reminder_key]
                rospy.loginfo(f"Reminder found: {reminder_text}")
                self.say_reminder(reminder_text)
            else:
                rospy.loginfo("Reminder not found")
            return

        rospy.loginfo("Unknown command")

    def get_point_name_from_command(self, command, prefix):
        """Извлечение названия точки из команды."""
        if command.startswith(prefix):
            point_name = command[len(prefix):].strip()
            if point_name in self.points_config['points']:
                return point_name
        return None

    def is_inspect_command(self, command):
        """Проверка команды "осмотрись"."""
        return command.strip() == "осмотрись"

    def perform_inspection(self):
        """Выполнение осмотра."""
        angles = [-15, 30, -30, 15]  # Углы поворота в градусах
        for angle in angles:
            self.rotate(angle)
            rospy.sleep(1)  # Пауза между поворотами
        rospy.sleep(2)  # Пауза после завершения осмотра
        self.return_to_start_point()

    def rotate(self, angle):
        """Выполнение поворота на заданный угол."""
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

    def say_reminder(self, text):
        """Озвучивание напоминания и возвращение в стартовую точку."""
        try:
            # Озвучиваем напоминание
            self.speech_service.call(SpeechRequest(data=text))
            rospy.loginfo(f"Reminder spoken: {text}")
            
            # Возвращаемся в стартовую точку
            rospy.sleep(1)  # Пауза для завершения озвучивания
            self.return_to_start_point()
        except Exception as e:
            rospy.logerr(f"Failed to speak reminder: {e}")

    def return_to_start_point(self):
        """Перемещение в стартовую точку."""
        rospy.loginfo("Returning to start point")
        goal = self._goal_message_assemble(self.start_point)
        self.client.send_goal(goal, done_cb=self.return_home_cb)
        self.returning_home = True

    def _goal_message_assemble(self, point):
        """Создание цели для move_base."""
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

    def move_base_cb(self, status, result):
        """Обратный вызов для move_base."""
        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Movement cancelled")
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Movement succeeded")
            if not self.returning_home:
                rospy.loginfo("Robot has arrived at the destination and is waiting for a new command")

    def return_home_cb(self, status, result):
        """Обратный вызов для возвращения в стартовую точку."""
        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Returning to start point cancelled")
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Returned to start point")
        self.returning_home = False

    def on_shutdown(self):
        """Действия при завершении работы."""
        rospy.loginfo("Shutting down voice navigation")
        self.cmd_pub.publish(Twist())  # Остановка движения
        self.client.cancel_all_goals()  # Отмена всех целей
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        rospy.init_node('voice_nav_node')
        robot = VoiceNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        robot.on_shutdown()
        rospy.loginfo("Voice navigation stopped due to ROS interrupt")