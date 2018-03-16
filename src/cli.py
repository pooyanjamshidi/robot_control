import argparse
import psutil
import rospy
from roslaunch import *
from roslaunch import rlutil, parent
import roslaunch
from bot_interface import ControlInterface
from bot_controller import BotController

commands = ["baseline_a", "baseline_b", "baseline_c"]
rosnode = "cp1_node"
launch_configs = {
    'default': 'cp1-base-test.launch'
}
launch_file_path = '~/catkin_ws/src/cp1_base/launch/'

# the starting waypoint
start_loc = ['l1']
# the target waypoints
target_locs = ['l2', 'l4', 'l2', 'l5', 'l8']


def launch(config, init_node="cp1_node"):
    launch_file = launch_configs[config]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid=uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path + launch_file])
    launch.start()


def init():
    rospy.init_node(rosnode)
    rospy.on_shutdown(stop)


def stop():
    '''kill gazebo'''
    for proc in psutil.process_iter():
        if proc.name() == "gzserver":
            proc.kill()


def baselineA():
    init()
    ci = ControlInterface()
    bot = BotController()


def baselineB():
    pass


def baselineC():
    pass


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=commands, help='The command to issue to Gazebo')

    args = parser.parse_args()

    if args.command == "baseline_a":
        baselineA()
    elif args.command == "baseline_b":
        baselineB()
    elif args.command == "baseline_c":
        baselineC()


if __name__ == '__main__':
    main()

