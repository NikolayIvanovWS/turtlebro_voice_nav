#!/usr/bin/env python3

from config_loader import ConfigLoader
from movement_manager import MovementManager
from command_handler import CommandHandler
import rospy
from std_msgs.msg import String
from turtlebro_speech.srv import Speech

class VoiceNav:
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)

        # Загружаем конфигурационные файлы
        self.config_loader = ConfigLoader()
        self.points_config = self.config_loader.load_points_config(rospy.get_param('~points_config_file'))
        self.commands_config = self.config_loader.load_commands_config(rospy.get_param('~commands_config_file'))
        self.speech_config = self.config_loader.load_speech_config(rospy.get_param('~speech_config_file'))

        # Инициализация клиента для сервиса озвучивания
        self.speech_service = rospy.ServiceProxy('festival_speech', Speech)
        rospy.loginfo("Ожидание запуска сервиса festival_speech")
        self.speech_service.wait_for_service()
        rospy.loginfo("Сервис festival_speech доступен")

        # Инициализация других компонентов
        self.movement_manager = MovementManager()
        self.command_handler = CommandHandler(
            self.points_config,
            self.commands_config,
            self.speech_config,
            self.movement_manager,
            self.speech_service  # Передаем speech_service в CommandHandler
        )

        # Подписываемся на топик с голосовыми командами
        self.voice_sub = rospy.Subscriber(
            '/robohead_controller/voice_recognizer_pocketsphinx/cmds_recognizer/commands',
            String,
            self.voice_command_cb
        )
        rospy.loginfo("Подписан на топик голосовых команд: /robohead_controller/voice_recognizer_pocketsphinx/cmds_recognizer/commands")

    def voice_command_cb(self, message):
        """Обработка голосовых команд."""
        command = message.data.strip()
        rospy.loginfo(f"Получена команда: {command}")
        self.command_handler.handle_command(command)

    def on_shutdown(self):
        """Действия при завершении работы."""
        rospy.loginfo("Завершение работы системы голосовой навигации")

if __name__ == '__main__':
    try:
        rospy.init_node('voice_nav_node')
        robot = VoiceNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Работа системы голосовой навигации остановлена из-за прерывания ROS")