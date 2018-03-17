import argparse
import psutil
from operator import itemgetter

import rospy
import math
import heapq
from roslaunch import *
from roslaunch import rlutil, parent
import roslaunch
from bot_interface import ControlInterface
from bot_controller import BotController

commands = ["baseline_a", "baseline_b", "baseline_c", "place_obstacle", "remove_obstacle"]
rosnode = "cp1_node"
launch_configs = {
    'default': 'cp1-base-test.launch'
}
launch_file_path = '~/catkin_ws/src/cp1_base/launch/'

# the starting waypoint
start = ['l1']
# the target waypoints
targets = ['l2', 'l4', 'l2', 'l5', 'l8']


def distance(loc1, loc2):
    return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)


def launch_cp1_base(config, init_node="cp1_node"):
    launch_file = launch_configs[config]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid=uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path + launch_file])
    launch.start()
    return launch


def init(node):
    rospy.init_node(node)
    rospy.on_shutdown(stop)


def stop(launch):
    '''kill gazebo'''
    launch.shutdown()

    for proc in psutil.process_iter():
        if proc.name() == "gzserver":
            proc.kill()


def baselineA(bot):
    init(rosnode)
    launch = launch_cp1_base('default')
    mission_time_predicted = bot.predict_mission_time(start, targets)
    start_time = rospy.Time.now()
    task_finished = bot.go_instructions_multiple_tasks(start, targets)
    finish_time = rospy.Time.now()
    mission_time_actual = finish_time - start_time

    x, y, w, v = bot.gazebo.get_bot_state()
    charge = bot.gazebo.get_battery_charge()
    target_loc = bot.map_server.waypoint_to_coords(targets[-1])
    distance_to_target = distance([x, y], [target_loc['x'], target_loc['y']])

    print("The bot finished {0} tasks in the mission and the current battery level is {1}Ah \n".format(task_finished, charge))
    print("The robot currently positioned at: x={0}, y={1}".format(x, y))

    stop(launch)


def baselineB(bot):
    pass


def baselineC(bot):
    pass


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=commands, help='The command to issue to Gazebo')
    bot = BotController()

    waypoints_locs = {}
    waypoints = bot.map_server.get_waypoints()
    for waypoint in waypoints:
        loc = bot.map_server.waypoint_to_coords(waypoint)
        waypoints_locs[waypoint] = loc

    args = parser.parse_args()

    if args.command == "baseline_a":
        baselineA(bot)
    elif args.command == "baseline_b":
        baselineB(bot)
    elif args.command == "baseline_c":
        baselineC(bot)
    elif args.command == "place_obstacle":
        # get the current position of the bot and place an obstacle in front few meters away
        x, y, w, v = bot.gazebo.get_bot_state()
        distances_to_locs = {}
        for waypoint in waypoints:
            loc = bot.map_server.waypoint_to_coords(waypoint)
            d = distance([x, y], [loc['x'], loc['y']])
            distances_to_locs[waypoint] = d
            
        #  place two obstacles on the closes waypoints to the current location of the robot
        two_closest_locs = heapq.nsmallest(2, distances_to_locs.iteritems(), itemgetter(1))
        loc1 = bot.map_server.waypoint_to_coords(two_closest_locs[0][0])
        bot.gazebo.place_obstacle(loc1['x'], loc1['y'])
        loc2 = bot.map_server.waypoint_to_coords(two_closest_locs[1][0])
        bot.gazebo.place_obstacle(loc2['x'], loc2['y'])

    elif args.command == "remove_obstacle":
        obstacles = bot.gazebo.obstacles
        for obstacle in obstacles:
            bot.gazebo.remove_obstacle(obstacle)


if __name__ == '__main__':
    main()

