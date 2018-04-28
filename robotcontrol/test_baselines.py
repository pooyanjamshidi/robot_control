#! /usr/bin/env python

# general imports
import argparse
import psutil
import math
import os
import time

# import ros libraries
from time import sleep

import rospy
from roslaunch import rlutil, parent
import roslaunch
from bot_controller import BotController
from constants import AdaptationLevel
from ready_db import ReadyDB

commands = ["baseline_a", "baseline_b", "baseline_c", "place_obstacle", "remove_obstacle"]
rosnode = "cp1_node"
launch_configs = {
    'default': 'cp1-base-test.launch'
}
launch_file_path = "~/catkin_ws/src/cp1_base/launch/"

ready_json = os.path.expanduser("~/ready")

# the starting waypoint
start = 'l1'
# the target waypoints
targets = ['l2', 'l4', 'l2', 'l5', 'l8']


def distance(loc1, loc2):
    return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)


def launch_cp1_base(config=None):
    if config is None:
        config = 'default'
    launch_file = launch_configs[config]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid=uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.expanduser(launch_file_path + launch_file)])
    launch.start()

    return launch


def init(node):
    rospy.init_node(node, anonymous=True)
    rospy.on_shutdown(graceful_stop)


def stop(launch):
    launch.shutdown()

    for proc in psutil.process_iter():
        if proc.name() == "gzserver":
            proc.kill()


def graceful_stop():
    rospy.logdebug("shutting down!")


def baselineA(bot, start, targets):
    launch = launch_cp1_base('default')

    # track battery charge
    bot.gazebo.track_battery_charge()

    #  sleep for few sec to bring up gazebo process properly
    sleep(10)

    # put the robot at the start position
    start_coords = bot.map_server.waypoint_to_coords(start)
    bot.gazebo.set_bot_position(start_coords['x'], start_coords['y'], 0)

    mission_time_predicted = bot.predict_mission_time(start, targets)

    start_time = time.time()
    rospy.loginfo("A mission with {0} tasks has been launched".format(len(targets)))
    task_finished, locs = bot.go_instructions_multiple_tasks_reactive(start, targets)
    finish_time = time.time()

    mission_time_actual = finish_time - start_time

    x, y, w, v = bot.gazebo.get_bot_state()
    charge = bot.gazebo.battery_charge
    target_loc = bot.map_server.waypoint_to_coords(targets[-1])
    distance_to_target = distance([x, y], [target_loc['x'], target_loc['y']])

    rospy.loginfo("The bot finished {0} tasks in the mission and the current battery level is {1}Ah \n".format(task_finished, charge))
    rospy.loginfo("The robot currently positioned at: x={0}, y={1}".format(x, y))
    rospy.loginfo("The mission was finished in {0} seconds, while it was predicted to finish in {1} seconds".format(mission_time_actual, mission_time_predicted))

    stop(launch)


def baselineB(bot, start, targets):
    launch = launch_cp1_base('default')

    # track battery charge
    bot.gazebo.track_battery_charge()

    #  sleep for few sec to bring up gazebo process properly
    sleep(10)

    # put the robot at the start position
    start_coords = bot.map_server.waypoint_to_coords(start)
    bot.gazebo.set_bot_position(start_coords['x'], start_coords['y'], 0)

    mission_time_predicted = bot.predict_mission_time(start, targets)

    #  place an obstacle before start
    # a_waypoint = bot.map_server.get_random_waypoint()
    a_waypoint = 'l3'
    loc = bot.map_server.waypoint_to_coords(a_waypoint)
    bot.gazebo.place_obstacle(loc['x'], loc['y'])

    start_time = time.time()
    rospy.loginfo("A mission with {0} tasks has been launched".format(len(targets)))
    task_finished, locs = bot.go_instructions_multiple_tasks_reactive(start, targets)
    finish_time = time.time()

    mission_time_actual = finish_time - start_time

    x, y, w, v = bot.gazebo.get_bot_state()
    charge = bot.gazebo.battery_charge
    target_loc = bot.map_server.waypoint_to_coords(targets[-1])
    distance_to_target = distance([x, y], [target_loc['x'], target_loc['y']])

    rospy.loginfo("The bot finished {0} tasks in the mission and the current battery level is {1}Ah \n".format(task_finished,
                                                                                                       charge))
    rospy.loginfo("The robot currently positioned at: x={0}, y={1}".format(x, y))
    rospy.loginfo("The mission was finished in {0} seconds, while it was predicted to finish in {1} seconds".format(
        mission_time_actual, mission_time_predicted))

    stop(launch)


def baselineC(bot, start, targets):
    launch = launch_cp1_base('default')

    # track battery charge
    bot.gazebo.track_battery_charge()

    #  sleep for few sec to bring up gazebo process properly
    sleep(10)

    # put the robot at the start position
    start_coords = bot.map_server.waypoint_to_coords(start)
    bot.gazebo.set_bot_position(start_coords['x'], start_coords['y'], 0)

    mission_time_predicted = bot.predict_mission_time(start, targets)

    #  place an obstacle before start
    # a_waypoint = bot.map_server.get_random_waypoint()
    a_waypoint = 'l3'
    loc = bot.map_server.waypoint_to_coords(a_waypoint)
    bot.gazebo.place_obstacle(loc['x'], loc['y'])

    start_time = time.time()
    rospy.loginfo("A mission with {0} tasks has been launched".format(len(targets)))
    task_finished, locs = bot.go_instructions_multiple_tasks_adaptive(start, targets)
    finish_time = time.time()

    mission_time_actual = finish_time - start_time

    x, y, w, v = bot.gazebo.get_bot_state()
    charge = bot.gazebo.battery_charge
    target_loc = bot.map_server.waypoint_to_coords(targets[-1])
    distance_to_target = distance([x, y], [target_loc['x'], target_loc['y']])

    rospy.loginfo(
        "The bot finished {0} tasks in the mission and the current battery level is {1}Ah \n".format(task_finished,
                                                                                                     charge))
    rospy.loginfo("The robot currently positioned at: x={0}, y={1}".format(x, y))
    rospy.loginfo("The mission was finished in {0} seconds, while it was predicted to finish in {1} seconds".format(
        mission_time_actual, mission_time_predicted))

    stop(launch)


def main():
    ready = ReadyDB(ready_db=ready_json)
    baseline = ready.get_baseline()
    start = ready.get_start_location()
    targets = ready.get_target_locations()

    # bring up a ros node
    init(rosnode)

    bot = BotController()

    if baseline == AdaptationLevel.BASELINE_A:
        baselineA(bot, start, targets)

    elif baseline == AdaptationLevel.BASELINE_B:
        baselineB(bot, start, targets)

    elif baseline == AdaptationLevel.BASELINE_C:
        baselineC(bot, start, targets)


if __name__ == '__main__':
    main()

