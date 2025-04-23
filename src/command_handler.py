#!/usr/bin/env python3

import rospy 
import math
from turtlebro_speech.srv import Speech, SpeechRequest

class CommandHandler:
    def __init__(self, points_config, commands_config, speech_config, movement_manager, speech_service):
        self.points_config = points_config
        self.commands_config = commands_config
        self.speech_config = speech_config
        self.movement_manager = movement_manager
        self.speech_service = speech_service
        
        self.photo_phrase = "сделай фото"

    def handle_command(self, command):
        """Обработка голосовых команд."""
        # Проверка команды "заверши работу"
        if command.strip() == "заверши работу":
            rospy.loginfo("Получена команда 'заверши работу'. Завершение работы...")
            self.shutdown_package()
            return

        # Проверка команды на перемещение
        for phrase in self.commands_config['movement_phrases']['phrases']:
            if command.startswith(phrase):
                point_name = self.get_point_name_from_command(command, phrase)
                if point_name:
                    rospy.loginfo(f"Перемещение к точке: {point_name}")
                    self.movement_manager.send_goal(self.points_config['points'][point_name])
                    return

        # Проверка команды "осмотрись"
        if self.is_inspect_command(command):
            if not hasattr(self, 'is_inspecting') or not self.is_inspecting:
                rospy.loginfo("Начинается последовательность осмотра")
                self.is_inspecting = True
                self.perform_inspection()
            return

        # Проверка команды "напомни"
        if command.startswith("напомни"):
            reminder_key = command[len("напомни"):].strip()
            if reminder_key in self.speech_config['reminders']:
                reminder_text = self.speech_config['reminders'][reminder_key]
                rospy.loginfo(f"Найдено напоминание: {reminder_text}")
                self.say_reminder(reminder_text)
            else:
                rospy.loginfo("Напоминание не найдено")
            return
            
        # Проверка команды "сделай фото"
        if self.photo_phrase in command:
            self.handle_photo_command()
            return

        rospy.loginfo("Неизвестная команда")

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
            self.movement_manager.rotate_to_angle(angle)
            rospy.sleep(1)  # Пауза между поворотами
        rospy.sleep(2)  # Пауза после завершения осмотра
        self.return_to_start_point()  # Возвращение в стартовую точку
        self.is_inspecting = False  # Сброс флага выполнения осмотра

    def return_to_start_point(self):
        """Перемещение в стартовую точку."""
        rospy.loginfo("Возвращение в стартовую точку")
        start_point = self.points_config['points'].get('стартовуюточку')
        if start_point:
            self.movement_manager.send_goal(start_point)
        else:
            rospy.logerr("Стартовая точка не найдена в конфигурации точек")

    def say_reminder(self, text):
        """Озвучивание напоминания и возвращение в стартовую точку."""
        try:
            rospy.loginfo(f"Попытка произнести напоминание: {text}")
            # Озвучиваем напоминание
            response = self.speech_service.call(SpeechRequest(data=text))
            if response.success:  # Проверка успешности выполнения
                rospy.loginfo(f"Напоминание успешно озвучено: {text}")
            else:
                rospy.logwarn("Сервис озвучивания не смог завершить запрос")
        except Exception as e:
            rospy.logerr(f"Не удалось вызвать сервис озвучивания: {e}")
            return

        # Пауза для завершения озвучивания
        rospy.sleep(1)

        # Возвращаемся в стартовую точку
        self.return_to_start_point()

    def handle_photo_command(self):
        """Обработка команды 'сделай фото'."""
        rospy.loginfo("Команда: Сделай фото")
        try:
            self.speech_service.call(SpeechRequest(data="Три и и и и и и. Два а а а а а. Одиииииин. Чиииииииииииз"))
            rospy.sleep(1)
        except Exception as e:
            rospy.logerr(f"Не удалось произнести сообщение для фото: {e}")

        # Возвращаемся в стартовую точку
        self.return_to_start_point()
    
    def shutdown_package(self):
        """Завершение работы пакета."""
        rospy.loginfo("Завершение работы пакета...")
        try:
            # Озвучивание фразы перед завершением (опционально)
            self.speech_service.call(SpeechRequest(data="Завершаю работу. До свидания!"))
            rospy.sleep(2)  # Пауза для завершения озвучивания
        except Exception as e:
            rospy.logerr(f"Не удалось произнести сообщение о завершении: {e}")

        # Завершение работы узла ROS
        rospy.signal_shutdown("Завершение работы пакета по команде пользователя.")
