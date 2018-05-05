#! /usr/bin/env python

# general imports
import psutil
import os

import rospy
from roslaunch import rlutil, parent
import roslaunch


rosnode = "cp1_node"
launch_configs = {
    'default': 'cp1-base-test.launch'
}
launch_file_path = "~/catkin_ws/src/cp1_base/launch/"


def add_to_queue(func):
    def wrapper(q, *args, **kwargs):
        result = func(*args, **kwargs)
        q.put(result)
        return result
    return wrapper


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
