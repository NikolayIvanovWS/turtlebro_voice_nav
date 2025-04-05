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
        rospy.loginfo("Have move_base action")
        
        # Создаем издатель для управления скоростью
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # Подписываемся на топик с голосовыми командами
        self.voice_sub = rospy.Subscriber('/robohead_controller/voice_recognizer_pocketsphinx/cmds_recognizer/commands', String, self.voice_command_cb)
        
        # Загружаем конфигурационные файлы
        self.load_points_config()
        self.load_commands_config()

        # Проверяем наличие стартовой точки
        if 'стартовуюточку' in self.points_config['points']:
            self.start_point = self.points_config['points']['стартовуюточку']
        else:
            rospy.logerr("Стартовая точка не найдена в конфигурационном файле points.toml")
            rospy.signal_shutdown("Стартовая точка не найдена")

        # Инициализация переменных
        self.current_point = None  # Текущая комната
        self.goal_sent = False  # Флаг отправки цели
        self.returning_home = False  # Флаг возвращения в стартовую точку
        self.is_inspecting = False  # Флаг выполнения осмотра
        rospy.loginfo("Init done")

    def load_points_config(self):
        """Загрузка конфигурации комнат."""
        config_data_file = rospy.get_param('~points_config_file', str(
            Path(__file__).parent.absolute()) + '/../data/points.toml')
        rospy.loginfo(f"Loading points config file {config_data_file}")
        try:
            self.points_config = toml.load(config_data_file)
            rospy.loginfo(f"points config loaded: {self.points_config}")
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

    def voice_command_cb(self, message):
        """Обработка голосовых команд."""
        command = message.data.strip()
        rospy.loginfo(f"Received command: {command}")

        # Проверка команды на соответствие ключевым фразам для перемещения
        for phrase in self.commands_config['movement_phrases']['phrases']:
            if command.startswith(phrase):
                point_name = self.get_point_name_from_command(command, phrase)
                if point_name:
                    rospy.loginfo(f"Moving to point: {point_name}")
                    self.current_point = point_name  # Сохраняем текущую комнату
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

        rospy.loginfo("Unknown command")

    def get_point_name_from_command(self, command, prefix):
        """Извлечение названия комнаты из команды."""
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
        self.return_to_start()

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

    def return_to_start(self):
        """Возвращение в стартовую точку."""
        rospy.loginfo("Returning to start point after inspection")
        goal = self._goal_message_assemble(self.start_point)
        self.client.send_goal(goal, done_cb=self.return_home_cb)
        self.returning_home = True
        self.is_inspecting = False

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
            if self.current_point:
                rospy.loginfo(f"Arrived at point: {self.current_point}")

    def return_home_cb(self, status, result):
        """Обратный вызов для возвращения в стартовую точку."""
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Returned to start point")
        self.returning_home = False

    def on_shutdown(self):
        """Действия при завершении работы узла."""
        rospy.loginfo("Shutdown VoiceNav")
        self.cmd_pub.publish(Twist())
        self.client.action_client.stop()
        rospy.sleep(0.5)

    def spin(self):
        """Основной цикл ROS."""
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('voice_nav_node')
        robot = VoiceNav()
        robot.spin()
    except rospy.ROSInterruptException:
        robot.on_shutdown()
        rospy.loginfo("VoiceNav stopped due to ROS interrupt")