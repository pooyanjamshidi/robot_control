import rospy
from gazebo_msgs.srv import *
from std_msgs.msg import Float64

from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
import actionlib
import dynamic_reconfigure.client
import sys
import ast


# importing battery services
from brass_gazebo_battery.srv import *

from tf.transformations import euler_from_quaternion
import math


ros_node = '/battery_monitor_client'
model_name = '/battery_demo_model'
map_name = 'map'
max_waiting_time = 100

# Here we manage the world, bot, and control interface

class ControlInterface:

    def __init__(self):

        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Gazebo services
        self.set_charging_srv = rospy.ServiceProxy(ros_node + model_name + '/set_charging', SetCharging)
        self.set_charge_rate_srv = rospy.ServiceProxy(ros_node + model_name + '/set_charge_rate', SetChargingRate)
        self.set_charge_srv = rospy.ServiceProxy(ros_node + model_name + '/set_charge', SetCharge)
        self.set_powerload_srv = rospy.ServiceProxy(ros_node + model_name + '/set_power_load', SetLoad)

        self.battery_charge = -1

    def move_to_point(self, x, y):
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        while not ac.wait_for_server(rospy.Duration.from_sec(5)):
            rospy.loginfo("waiting for the action server")

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = map_name
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = Point(x, y, 0)

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        ac.send_goal(goal)
        success = ac.wait_for_result(rospy.Duration.from_sec(max_waiting_time))

        state = ac.get_state()

        if success and state == GoalStatus.SUCCEEDED
            rospy.loginfo("reached the destination")
            return True
        else:
            rospy.loginfo("could not reached the destination")
            return False




    def set_bot_position(self, x, y, w):
        pass

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
    # ci.set_charge(3)
    ci.set_charging(1)

    # monitor_battery()
    ci.move_to_point(1, 1)


if __name__ == '__main__':
    main()
