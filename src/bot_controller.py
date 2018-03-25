#! /usr/bin/env python

"""bot controller"""
import os
import time
import math
from mapserver import MapServer
from instructions_db import InstructionDB
from bot_interface import ControlInterface
from configuration_db import ConfigurationDB
import rospy


map_file = os.path.expanduser("~/catkin_ws/src/cp1_base/maps/cp1_map.json")
instructions_db_file = os.path.expanduser("~/catkin_ws/src/cp1_base/instructions/instructions-all.json")
config_list = os.path.expanduser("~/cp1/config/config_list.json")

sleep_interval = 5
distance_threshold = 1


def distance(loc1, loc2):
    return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)


class BotController:

    def __init__(self):
        self.map_server = MapServer(map_file)
        self.instruction_server = InstructionDB(instructions_db_file)
        self.gazebo = ControlInterface()
        self.config_server = ConfigurationDB(config_list)

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

    def update_speed(self, igcode):
        """updates the speed in the instruction based on the current configuration of the robot,
        note the way how configuration affect speed as a proxy in cp1"""
        current_config = self.gazebo.get_current_configuration(current_or_historical=True)


    def go_instructions(self, start, target):
        """bot execute the instructions and goes from start to the target with the directions instructed by the igcode

        :param start: start waypoint id
        :param target: target waypoint id
        :return:
        """

        # get the yaw (direction) where the robot is headed
        w = self.instruction_server.get_start_heading(start, target)
        if w == -1:
            rospy.logerr("No information for {0} to {1}".format(start, target))
            return False

        if self.gazebo.ig_client is None:
            self.gazebo.connect_to_ig_action_server()

        # get the x, y coordinates from the map server and put the robot there using the gazebo interface
        # start_coords = self.map_server.waypoint_to_coords(start)
        # self.gazebo.set_bot_position(start_coords['x'], start_coords['y'], w)

        # get the instruction code and execute it
        igcode = self.instruction_server.get_instructions(start, target)
        res = self.gazebo.move_bot_with_igcode(igcode)

        return res

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
            success = self.go_instructions(current_start, target)
            # low_charge = self.gazebo.ig_server.new_goal_preempt_request

            x, y, w, v = self.gazebo.get_bot_state()
            loc_target = self.map_server.waypoint_to_coords(target)

            #  check robot distance to the target, if it is not then
            d = distance([x, y], [loc_target['x'], loc_target['y']])
            if d > distance_threshold and success:
                rospy.logwarn("The robot is not close enough to the expected target, so we do not count this task done!")
                start = target
                success = False

            locs.append({"start": current_start, "target": target, "x": x, "y": y, "task_accomplished": success, "dist_to_target": d})

            if success:
                rospy.loginfo("A new task ({0}->{1}) has been accomplished".format(current_start, target))
                start = target
                number_of_tasks_accomplished += 1

            if self.gazebo.is_battery_low:
                bot_state = self.gazebo.get_bot_state()
                loc = {"x": bot_state[0], "y": bot_state[1]}
                res, charging_id = self.go_charging(loc)
                while not res:
                    res, charging_id = self.go_charging(loc)
                while not self.is_fully_charged():
                    time.sleep(sleep_interval)
                self.undock()
                start = charging_id

        return number_of_tasks_accomplished, locs

    def go_charging(self, current_loc):
        """bot goes to the closest charging station from the current waypoint it is on"""
        rospy.logwarn("The battery is low and the bot is now heading to the nearest charging station")
        current_waypoint = self.map_server.coords_to_waypoint(current_loc)['id']
        path_to_charging = self.map_server.closest_charging_station(current_waypoint)
        charging_id = path_to_charging[-1]
        res = self.go_without_instructions(charging_id)
        if res:
            self.dock()
        else:
            rospy.logwarn("The instruction to go to the nearest charging station was failed")
        return res, charging_id

    def is_fully_charged(self):
        if self.gazebo.battery_charge == self.gazebo.battery_capacity:
            rospy.loginfo("Battery is fully charged.")
            return True
        else:
            return False

    def dock(self):
        if self.gazebo.is_charging:
            rospy.logwarn("The bot is currently docked")
            return False
        else:
            rospy.loginfo("The bot is docked and start charging")
            self.gazebo.set_charging(1)
            return True

    def undock(self):
        if self.gazebo.is_charging:
            self.gazebo.set_charging(0)
            rospy.loginfo("The bot is now undocked")
            return True
        else:
            rospy.loginfo("The bot is not docked")
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

