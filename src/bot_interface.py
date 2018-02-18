import traceback
import math
import json
import numpy


# third party imports
import rospy
from std_msgs.msg import Float64
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
import actionlib
import dynamic_reconfigure.client
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import ig_action_msgs.msg
from ig_action_msgs.msg import InstructionGraphResult
from mars_notifications.msg import UserNotification
from kobuki_msgs.msg import MotorPower

# importing battery services
from brass_gazebo_battery.srv import *

# parameters and global variables
ros_node = '/battery_monitor_client'
model_name = '/battery_demo_model'
map_name = 'map'
max_waiting_time = 100

# the threshold below which the bot will go to the charging station
battery_low_threshold = 0.2

conf_file = '../conf/conf.json'

default_configuration_id = 0
default_power_load = 10

# Here we manage the world, bot, and control interface


class ControlInterface:

    def __init__(self):

        # standard Gazebo services
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Battery plugin Gazebo services
        self.set_charging_srv = rospy.ServiceProxy(ros_node + model_name + '/set_charging', SetCharging)
        self.set_charge_rate_srv = rospy.ServiceProxy(ros_node + model_name + '/set_charge_rate', SetChargingRate)
        self.set_charge_srv = rospy.ServiceProxy(ros_node + model_name + '/set_charge', SetCharge)
        self.set_powerload_srv = rospy.ServiceProxy(ros_node + model_name + '/set_power_load', SetLoad)

        # AMCL topic
        self.amcl = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)

        self.battery_charge = -1

        self.movebase_client = None
        self.ig_client = None

        self.bot_conf = None

    def read_conf(self):

        with open(conf_file) as json_file:
            self.bot_conf = json.load(json_file)

    def update_conf(self, conf):

        with open(conf_file, 'w') as json_file:
            json.dump(conf, json_file)

        # update the bot configuration
        self.bot_conf = conf

    def connect_to_navigation_server(self):

        self.movebase_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        while not self.movebase_client.wait_for_server(rospy.Duration.from_sec(5)):
            rospy.loginfo("waiting for the action server")

        rospy.loginfo("successfully connected to the action server")
        return True

    def move_to_point(self, x, y):

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = map_name
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = Point(x, y, 0)

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        self.movebase_client.send_goal(goal)
        success = self.movebase_client.wait_for_result(rospy.Duration.from_sec(max_waiting_time))

        state = self.movebase_client.get_state()

        if success and state == GoalStatus.SUCCEEDED:
            rospy.loginfo("reached the destination")
            return True
        else:
            rospy.loginfo("could not reached the destination")
            return False

    def connect_to_ig_action_server(self):

        self.ig_client = actionlib.SimpleActionClient("ig_action_server", ig_action_msgs.msg.InstructionGraphAction)

        while not self.ig_client.wait_for_server(rospy.Duration.from_sec(max_waiting_time)):
            rospy.loginfo("waiting for the ig_action_server")

        rospy.loginfo("successfully connected to the ig_action_server")
        return True

    def move_bot_with_ig(self, ig_file):

        with open(ig_file) as igfile:
            igcode = igfile.read()
            goal = ig_action_msgs.msg.InstructionGraphGoal(order=igcode)
            self.ig_client.send_goal(goal=goal)
            success = self.ig_client.wait_for_result(rospy.Duration.from_sec(max_waiting_time))

            state = self.ig_client.get_state()
            if success and state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Successfully executed the instructions and reached the destination")
                return True
            else:
                rospy.loginfo("could not execute the instructions")
                return False

    def move_bot_with_igcode(self, igcode):

        goal = ig_action_msgs.msg.InstructionGraphGoal(order=igcode)
        self.ig_client.send_goal(goal=goal)
        success = self.ig_client.wait_for_result(rospy.Duration.from_sec(max_waiting_time))

        state = self.ig_client.get_state()
        if success and state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully executed the instructions and reached the destination")
            return True
        else:
            rospy.loginfo("could not execute the instructions")
            return False

    def set_bot_position(self, x, y, w):

        try:
            tp = self.get_model_state('mobile_base', '')

            tp.pose.position.x = x
            tp.pose.position.y = y
            quat = (tp.pose.orientation.x, tp.pose.orientation.y, tp.pose.orientation.z, tp.pose.orientation.w)
            (roll, pitch, yaw) = euler_from_quaternion(quat)
            yaw = w
            quat = quaternion_from_euler(roll, pitch, yaw)

            tp.pose.orientation.x = quat[0]
            tp.pose.orientation.y = quat[1]
            tp.pose.orientation.z = quat[2]
            tp.pose.orientation.w = quat[3]

            ms = ModelState()
            ms.model_name = "mobile_base"
            ms.pose = tp.pose
            ms.twist = tp.twist

            result = self.set_model_state(ms)

            if result.success:
                ip = PoseWithCovarianceStamped()
                ip.header.stamp = rospy.Time.now()
                ip.header.frame_id = map_name
                ip.pose.pose.position.x = x
                ip.pose.pose.position.y = y
                ip.pose.pose.position.z = 0
                ip.pose.pose.orientation.x = tp.pose.orientation.x
                ip.pose.pose.orientation.y = tp.pose.orientation.y
                ip.pose.pose.orientation.z = tp.pose.orientation.z
                ip.pose.pose.orientation.w = tp.pose.orientation.w
                self.amcl.publish(ip)
                rospy.loginfo("The bot is positioned in the new place at ({0}, {1})".format(x, y))
                return True
            else:
                rospy.logerr("Error occurred putting the bot in the position")
                return False

        except rospy.ServiceException as e:
            rospy.logerr("Could not set the position of the bot")
            rospy.logerr(e.message)

    def get_bot_state(self):

        try:
            tp = self.get_model_state('mobile_base', '')
            quat = (tp.pose.orientation.x, tp.pose.orientation.y, tp.pose.orientation.z, tp.pose.orientation.w)
            (roll, pitch, yaw) = euler_from_quaternion(quat)
            v = math.sqrt(tp.twist.linear.x**2 + tp.twist.linear.y**2)
            return tp.pose.position.x, tp.pose.position.y, yaw, v

        except rospy.ServiceException as se:
            rospy.logerr("Error happened while getting bot position: %s", se)
            return None, None, None, None

    def set_charging(self, charging):
        return self.set_charging_srv(charging)

    def set_charge(self, charge):
        return self.set_charge_srv(charge)

    def set_power_load(self, load):
        return self.set_powerload_srv(load)

    def set_charging_rate(self, charge_rate):
        return self.set_charge_rate_srv(charge_rate)


def get_charge(msg):
    global battery_charge
    battery_charge = msg.data
    print(battery_charge)


def monitor_battery():
    rospy.init_node("battery_monitor_client")
    rospy.Subscriber("/mobile_base/commands/charge_level", Float64, get_charge)
    rospy.spin()


def main():

    global battery_charge
    battery_charge = -1

    rospy.init_node('navigation', anonymous=False)

    ci = ControlInterface()
    state = ci.get_bot_state()
    print("Bot is located at ({0}, {1}), facing {2} and going with a speed of {3} m/s".format(state[0], state[1], state[2], state[3]))

    # ci.set_power_load(1)
    ci.set_charge(1)
    ci.set_charging(1)

    # monitor_battery()
    # ci.move_to_point(0, -1)
    # ci.set_bot_position(0, 0, 0)

    ci.read_conf()

    ci.connect_to_ig_action_server()
    ci.move_bot_with_ig('../instructions/nav_test1.ig')


if __name__ == '__main__':
    main()
