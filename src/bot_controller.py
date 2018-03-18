"""bot controller"""
import os
import time
from mapserver import MapServer
from instructions_db import InstructionDB
from bot_interface import ControlInterface

map_file = os.path.expanduser("~/catkin_ws/src/cp1_base/maps/cp1_map.json")
instructions_db_file = os.path.expanduser("~/catkin_ws/src/cp1_base/instructions/instructions-all.json")
sleep_interval = 5


class BotController:

    def __init__(self):
        self.map_server = MapServer(map_file)
        self.instruction_server = InstructionDB(instructions_db_file)
        self.gazebo = ControlInterface()

    def go_without_instructions(self, target):
        """bot goes directly from start to the target using move base

        :param start: start waypoint id
        :param target: target waypoint id
        :return:
        """
        # initializing the move base client
        if self.gazebo.movebase_client is None:
            self.gazebo.connect_to_navigation_server()

        # get the x, y coordinates from the map server and put the robot there using the gazebo interface
        # start_coords = self.map_server.waypoint_to_coords(start)
        # self.gazebo.set_bot_position(start_coords['x'], start_coords['y'], 0)
        target_coords = self.map_server.waypoint_to_coords(target)

        # head to the target
        res = self.gazebo.move_to_point(target_coords['x'], target_coords['y'])

        return res

    def go_instructions(self, start, target):
        """bot execute the instructions and goes from start to the target with the directions instructed by the igcode

        :param start: start waypoint id
        :param target: target waypoint id
        :return:
        """

        # get the yaw (direction) where the robot is headed
        w = self.instruction_server.get_start_heading(start, target)
        if w == -1:
            print("No information for %s to %s".format(start, target))
            return False

        if self.gazebo.ig_client is None:
            self.gazebo.connect_to_ig_action_server()

        # get the x, y coordinates from the map server and put the robot there using the gazebo interface
        # start_coords = self.map_server.waypoint_to_coords(start)
        # self.gazebo.set_bot_position(start_coords['x'], start_coords['y'], w)

        # get the instruction code and execute it
        igcode = self.instruction_server.get_instructions(start, target)
        res, low_charge = self.gazebo.move_bot_with_igcode(igcode)

        return res, low_charge

    def go_instructions_multiple_tasks(self, start, targets):
        """the same version of go_instructions but for multiple tasks for cp1

        :param start:
        :param targets:
        :return:
        """
        number_of_tasks_accomplished = 0
        locs = []

        for target in targets:
            current_start = start
            success, low_charge = self.go_instructions(current_start, target)
            low_charge = self.gazebo.ig_server.new_goal_preempt_request

            x, y, w, v = self.gazebo.get_bot_state()
            locs.append({"start": current_start, "target": target, "x": x, "y": y, "task_accomplished": success})

            if success:
                start = target
                number_of_tasks_accomplished += 1

            if low_charge:
                bot_state = self.gazebo.get_bot_state()
                loc = {"x": bot_state[0], "y": bot_state[1]}
                res, charging_id = self.go_charging(loc)
                while not self.is_fully_charged():
                    time.sleep(sleep_interval)
                self.undock()
                start = charging_id

        return number_of_tasks_accomplished, locs

    def go_charging(self, current_loc):
        """bot goes to the closest charging station from the current waypoint it is on"""
        current_waypoint = self.map_server.coords_to_waypoint(current_loc)['id']
        path_to_charging = self.map_server.closest_charging_station(current_waypoint)
        charging_id = path_to_charging[-1]
        res = self.go_without_instructions(charging_id)
        if res:
            self.dock()
        return res, charging_id

    def is_fully_charged(self):
        if self.gazebo.battery_charge == self.gazebo.battery_capacity:
            return True
        else:
            return False

    def dock(self):
        if self.gazebo.is_charging:
            print("the bot is currently docked")
            return False
        else:
            self.gazebo.set_charging(1)
            return True

    def undock(self):
        if self.gazebo.is_charging:
            self.gazebo.set_charging(0)
            return True
        else:
            print("the bot is not docked")
            return False

    def predict_mission_time(self, start, targets):
        """predict mission time in baseline A and B, a mission is comprised on several tasks

        :return:
        """
        mission_time = 0
        for target in targets:
            current_start = start
            mission_time += self.instruction_server.get_predicted_duration(current_start, target)
            start = target

        return mission_time

    def update_bot_configuration(self):
        """updates the gazebo bot configuration and the power consumption in one place"""
        pass

