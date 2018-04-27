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

commands = ["place_obstacle", "remove_obstacle", "set_charge", "execute_task", "go_directly", "execute_task_reactive",
            "execute_task_reactive_fancy"]

rosnode = "cp1_node"
launch_configs = {
    'default': 'cp1-base-test.launch'
}
launch_file_path = "~/catkin_ws/src/cp1_base/launch/"


def launch_cp1_base(config):
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
