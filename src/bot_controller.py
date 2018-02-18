"""bot controller"""
import os
from mapserver import MapServer
from instructions_db import InstructionDB
from bot_interface import ControlInterface

map_file = os.path.expanduser("~/catkin_ws/src/cp1_base/maps/cp1_map.json")
instructions_db_file = os.path.expanduser("~/catkin_ws/src/cp1_base/instructions/instructions-all.json")


class BotController:

    def __init__(self):
        self.map_server = MapServer(map_file)
        self.instruction_server = InstructionDB(instructions_db_file)
        self.gazebo = ControlInterface()

    def go_without_instructions(self, start, target):
        """bot goes directly from start to the target using move base

        :param start: start waypoint id
        :param target: target waypoint id
        :return:
        """
        # initializing the move base client
        if self.gazebo.movebase_client is None:
            self.gazebo.connect_to_navigation_server()

        # get the x, y coordinates from the map server and put the robot there using the gazebo interface
        start_coords = self.map_server.waypoint_to_coords(start)
        self.gazebo.set_bot_position(start_coords['x'], start_coords['y'], 0)
        target_coords = self.map_server.waypoint_to_coords(target)

        # head to the target
        res = self.gazebo.move_to_point(start_coords, target_coords)

        return res

    def go_instructions(self, start, target):
        """bot execute the instructions and goes from start to the target with the directions instructed by the igcode

        :param start: start waypoint id
        :param target: target waypoint id
        :return:
        """
        w = self.instruction_server.get_start_heading(start, target)
        if w == -1:
            print("No information for %s to %s".format(start, target))
            return False

        if self.gazebo.ig_client is None:
            self.gazebo.connect_to_ig_action_server()

        # get the x, y coordinates from the map server and put the robot there using the gazebo interface
        start_coords = self.map_server.waypoint_to_coords(start)
        self.gazebo.set_bot_position(start_coords['x'], start_coords['y'], w)

        # get the instruction code and execute it
        igcode = self.instruction_server.get_instructions(start, target)
        self.gazebo.move_bot_with_igcode(igcode)
