#! /usr/bin/env python

"""robot mission-level controller"""
import os
import time
import math
import rospy
import re
from multiprocessing import Process
from threading import Thread

from robotcontrol.mapserver import MapServer
from robotcontrol.instructions_db import InstructionDB
from robotcontrol.bot_interface import ControlInterface
from robotcontrol.configuration_db import ConfigurationDB
from robotcontrol.battery_db import BatteryDB
from robotcontrol.constants import AdaptationLevel


map_file = os.path.expanduser("~/catkin_ws/src/cp1_base/maps/cp1_map.json")
instructions_db_file = os.path.expanduser("~/catkin_ws/src/cp1_base/instructions/instructions-all.json")
config_list = os.path.expanduser("~/cp1/config_list_true.json")
world_file = os.path.expanduser("~/catkin_ws/src/cp1_base/worlds/p2-cp1-1.world")

battery_name = "brass_battery"
sleep_interval = 5
distance_threshold = 2

# for Rainbow integration
current_target_waypoint = os.path.expanduser("~/cp1/current-target-waypoint")
current_task_finished = os.path.expanduser("~/cp1/current-task-finished")


def distance(loc1, loc2):
    return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)


class BotController:

    def __init__(self):
        self.map_server = MapServer(map_file)
        self.instruction_server = InstructionDB(instructions_db_file)
        self.config_server = ConfigurationDB(config_list)
        self.robot_battery = BatteryDB(world_file, battery_name=battery_name)
        self.gazebo = ControlInterface(self.config_server.get_default_config())
        self.level = None

        # robot initialization including the battery, etc
        self.init_robot()

    def init_robot(self):
        self.gazebo.battery_capacity = self.robot_battery.capacity
        self.gazebo.charge_rate = self.robot_battery.charge_rate
        self.gazebo.battery_voltage = self.robot_battery.battery_voltage

    def go_without_instructions(self, target):
        """bot goes directly from start to the target using move base

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
        new_speed = self.config_server.get_speed(current_config)
        matches = re.findall('MoveAbsH\(([-+]?\d*\.\d+), ([-+]?\d*\.\d+|\d+), ([-+]?\d*\.\d+|\d+)', igcode)
        # replace the third value in MoveAbsH with the new speed value
        igcode_updated = igcode.replace(matches[0][2], str(new_speed))

        return igcode_updated

    def go_instructions(self, start, target, wait=True, active_cb=None, done_cb=None):
        """bot execute the instructions and goes from start to the target with the directions instructed by the igcode

        :param start: start waypoint id
        :param target: target waypoint id
        :param wait:
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

        # get the instruction code, update the speed based on current configuration and execute it
        igcode = self.instruction_server.get_instructions(start, target)
        # update the speed to reflect the influence of configuration
        updated_igcode = self.update_speed(igcode=igcode)

        if wait:
            res = self.gazebo.move_bot_with_igcode(updated_igcode, active_cb=active_cb, done_cb=done_cb)
            return res
        else:
            self.gazebo.send_instructions(igcode=igcode, active_cb=active_cb, done_cb=done_cb)
            return True

    def start(self, start, targets, active_cb=None, done_cb=None, at_waypoint_cb=None, mission_done_cb=None):
        """this is an interface for the mission sequencer"""

        rospy.loginfo("Starting the mission!")

        if self.level == "c":
            t = Thread(target=self.go_instructions_multiple_tasks_adaptive,
                        args=(start, targets, active_cb, done_cb, at_waypoint_cb, mission_done_cb))
            t.start()
        else:
            t = Thread(target=self.go_instructions_multiple_tasks_reactive,
                        args=(start, targets, active_cb, done_cb, at_waypoint_cb, mission_done_cb))
            t.start()

    def go_instructions_multiple_tasks_reactive(self, start, targets, active_cb=None, done_cb=None, at_waypoint_cb=None, mission_done_cb=None):
        """the same version of go_instructions but for multiple tasks for cp1

        :param start:
        :param targets:
        :return:
        """
        rospy.loginfo("Reactive robot started the mission!")

        number_of_tasks_accomplished = 0
        locs = []

        for target in targets:
            rospy.loginfo("Starting a new task to get to: {}".format(target))
            current_start = start
            success = self.go_instructions(current_start, target, wait=True, active_cb=active_cb, done_cb=done_cb)

            x, y, w, v = self.gazebo.get_bot_state()
            loc_target = self.map_server.waypoint_to_coords(target)

            #  check robot distance to the target, if it is not then
            d = distance([x, y], [loc_target['x'], loc_target['y']])
            if d > distance_threshold and success:
                rospy.logwarn(
                    "The robot is not close enough to the expected target, so we do not count this task done!")
                start = target
                success = False
            elif d <= distance_threshold and not success:
                rospy.logwarn(
                    "Apparently the robot could accomplish the task but ig_server reported differently!")
                start = target
                success = True

            if at_waypoint_cb is not None:
                at_waypoint_cb(target)

            locs.append({"start": current_start, "target": target, "x": x, "y": y, "task_accomplished": success, "dist_to_target": d})

            if success:
                rospy.loginfo("A new task ({0}->{1}) has been accomplished".format(current_start, target))
                start = target
                number_of_tasks_accomplished += 1
            else:
                rospy.logwarn("The task ({0}->{1}) has been failed".format(current_start, target))

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

        if mission_done_cb is not None:
            mission_done_cb(number_of_tasks_accomplished, locs)

        return number_of_tasks_accomplished, locs

    def go_instructions_multiple_tasks_reactive_fancy(self, start, targets, active_cb=None, done_cb=None, at_waypoint_cb=None, mission_done_cb=None):
        """the same version of reactive but with configuration adaptation

        :param start:
        :param targets:
        :return:
        """

        rospy.loginfo("Reactive robot (fancy) started the mission!")

        number_of_tasks_accomplished = 0
        locs = []

        for target in targets:
            rospy.loginfo("Starting a new task to get to: {}".format(target))
            current_start = start
            success = self.go_instructions(current_start, target, wait=True, active_cb=active_cb, done_cb=done_cb)

            x, y, w, v = self.gazebo.get_bot_state()
            loc_target = self.map_server.waypoint_to_coords(target)

            #  check robot distance to the target, if it is not then
            d = distance([x, y], [loc_target['x'], loc_target['y']])
            if d > distance_threshold and success:
                rospy.logwarn(
                    "The robot is not close enough to the expected target, so we do not count this task done!")
                start = target
                success = False
            elif d <= distance_threshold and not success:
                rospy.logwarn(
                    "Apparently the robot could accomplish the task but ig_server reported differently!")
                start = target
                success = True

            if at_waypoint_cb is not None:
                at_waypoint_cb(target)

            locs.append({"start": current_start, "target": target, "x": x, "y": y, "task_accomplished": success, "dist_to_target": d})

            if success:
                rospy.loginfo("A new task ({0}->{1}) has been accomplished".format(current_start, target))
                start = target
                number_of_tasks_accomplished += 1
            else:
                rospy.logwarn("The task ({0}->{1}) has been failed".format(current_start, target))

            if self.gazebo.is_battery_low:
                bot_state = self.gazebo.get_bot_state()
                loc = {"x": bot_state[0], "y": bot_state[1]}

                # check whether an adaptation (e.g., change of configuration) is needed
                if self.can_bot_reach_charging(loc):
                    self.adapt(AdaptationLevel.BASELINE_C)

                res, charging_id = self.go_charging(loc)
                while not res:
                    res, charging_id = self.go_charging(loc)
                while not self.is_fully_charged():
                    time.sleep(sleep_interval)
                self.undock()
                start = charging_id

        if mission_done_cb is not None:
            mission_done_cb(number_of_tasks_accomplished, locs)

        return number_of_tasks_accomplished, locs

    def go_instructions_multiple_tasks_adaptive(self, start, targets, active_cb=None, done_cb=None, at_waypoint_cb=None, mission_done_cb=None):
        """this is for baseline c where the adaptation will be taken care of with Rainbow"""

        rospy.loginfo("Adaptive robot started the mission!")

        number_of_tasks_accomplished = 0
        locs = []

        for target in targets:
            rospy.loginfo("Starting a new task to get to: {}".format(target))
            current_start = start
            self.update_current_target_waypoint_and_resetting_previous(current_waypoint=target)
            self.go_instructions(current_start, target, wait=False, active_cb=active_cb, done_cb=done_cb)

            success = self.wait_until_rainbow_is_done()

            x, y, w, v = self.gazebo.get_bot_state()
            loc_target = self.map_server.waypoint_to_coords(target)

            #  check robot distance to the target, if it is not then
            d = distance([x, y], [loc_target['x'], loc_target['y']])
            if d > distance_threshold and success:
                rospy.logwarn(
                    "The robot is not close enough to the expected target, so we do not count this task done!")
                start = target
                success = False
            elif d <= distance_threshold and not success:
                rospy.logwarn(
                    "Apparently the robot could accomplish the task but ig_server reported differently!")
                start = target
                success = True

            if at_waypoint_cb is not None:
                at_waypoint_cb(target)

            locs.append({"start": current_start, "target": target, "x": x, "y": y, "task_accomplished": success,
                         "dist_to_target": d})

            if success:
                rospy.loginfo("The task ({0}->{1}) has been accomplished".format(current_start, target))
                start = target
                number_of_tasks_accomplished += 1
            else:
                rospy.logwarn("The task ({0}->{1}) has been failed".format(current_start, target))

        if mission_done_cb is not None:
            mission_done_cb(number_of_tasks_accomplished, locs)

        return number_of_tasks_accomplished, locs

    def wait_until_rainbow_is_done(self):
        """Rainbow should indicate when it thinks it is done with the task"""
        while True:
            current_task_file = open(current_task_finished, "r")
            res = current_task_file.read().replace('\n', '')
            current_task_file.close()
            if res == "DONE":
                return True
            elif res == "FAILED":
                return False
            else:
                time.sleep(sleep_interval)

    def update_current_target_waypoint_and_resetting_previous(self, current_waypoint):
        """update a shared file to inform rainbow about current waypoint"""
        with open(current_target_waypoint, "w+") as file:
            file.write("target: \"%s\"" %current_waypoint)
        open(current_task_finished, "w+").close()

    def adapt(self, adaptation_level):
        """adaptation factory"""
        if adaptation_level == AdaptationLevel.BASELINE_C:
            self.change_config_to_conservative()

    def change_config_to_conservative(self):
        conservative_config = self.config_server.get_a_conservative_config()
        self.gazebo.set_current_configuration(config_id=conservative_config)

    def go_charging(self, current_loc):
        """bot goes to the closest charging station from the current waypoint it is on"""
        rospy.logwarn("The bot is now heading to the nearest charging station")
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

    def can_bot_reach_charging(self, current_loc):
        """use the power model to check whether the robot low on battery can reach the closest charging station"""
        charge_level = self.gazebo.battery_charge
        power_load = self.config_server.get_power_load(self.gazebo.current_config)
        dischagre_time = self.robot_battery.time_to_fully_discharge(charge_level, power_load)

        current_speed = self.config_server.get_speed(self.gazebo.current_config)
        current_waypoint = self.map_server.coords_to_waypoint(current_loc)['id']
        path_to_charging = self.map_server.closest_charging_station(current_waypoint)

        next_loc = self.map_server.waypoint_to_coords(path_to_charging[0])
        dist_to_charging = distance([current_loc['x'], current_loc['y']], [next_loc['x'], next_loc['y']])
        for waypoint in path_to_charging[1:-1]:
            next_loc = self.map_server.waypoint_to_coords(waypoint)
            dist_to_charging += distance([current_loc['x'], current_loc['y']], [next_loc['x'], next_loc['y']])
            current_loc = next_loc
        travel_time_to_charging = dist_to_charging / current_speed

        if dischagre_time >= travel_time_to_charging:
            return True
        else:
            return False
