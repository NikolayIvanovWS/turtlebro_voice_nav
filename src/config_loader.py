#!/usr/bin/env python3

import rospy
import toml
from pathlib import Path

class ConfigLoader:
    def __init__(self):
        pass

    def load_points_config(self, config_file):
        """Загрузка конфигурации точек."""
        try:
            return toml.load(config_file)
        except Exception as e:
            rospy.logerr(f"Не удалось загрузить конфигурацию точек: {e}")
            rospy.signal_shutdown("Неверный файл конфигурации точек TOML")

    def load_commands_config(self, config_file):
        """Загрузка конфигурации ключевых фраз."""
        try:
            return toml.load(config_file)
        except Exception as e:
            rospy.logerr(f"Не удалось загрузить конфигурацию команд: {e}")
            rospy.signal_shutdown("Неверный файл конфигурации команд TOML")

    def load_speech_config(self, config_file):
        """Загрузка конфигурации напоминаний."""
        try:
            return toml.load(config_file)
        except Exception as e:
            rospy.logerr(f"Не удалось загрузить конфигурацию напоминаний: {e}")
            rospy.signal_shutdown("Неверный файл конфигурации напоминаний TOML")