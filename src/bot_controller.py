"""bot controller"""
import os
from mapserver import MapServer
from instructions_db import InstructionDB
from bot_interface import ControlInterface

map_file = os.path.expanduser("~/catkin_ws/src/cp1_base/maps/cp1_map.json")
instructions_path = os.path.expanduser("~/catkin_ws/src/cp1_base/instructions/")


class BotController:

    def __init__(self):
        self.map_server = MapServer(map_file)
        self.instruction_server = InstructionDB(instructions_path)
        self.gazebo = ControlInterface()

    def go_directly(self, start, target= None):
        pass

    def go_instructions(self, start, target):
        pass

    def exec_instructions(self, start, end):
        pass
