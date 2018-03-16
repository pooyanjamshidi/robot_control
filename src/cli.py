import argparse
import rospy
from bot_interface import ControlInterface
from bot_controller import BotController

commands = ["baseline_a", "baseline_b", "baseline_c"]
rosnode = "cp1_node"


def baselineA():
    rospy.init_node(rosnode)
    ci = ControlInterface()
    bot = BotController()




def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=commands, help='The command to issue to Gazebo')

    args = parser.parse_args()

    if args.command == "baseline_a":
        baselineA()


if __name__ == '__main__':
    main()

