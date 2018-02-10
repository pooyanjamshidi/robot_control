import rospy
from gazebo_msgs.srv import *
from std_msgs.msg import Float64

# importing battery services
from brass_gazebo_battery.srv import *

from tf.transformations import euler_from_quaternion
import math


ros_node = '/battery_monitor_client'
model_name = '/battery_demo_model'

# Here we manage the world, bot, and control interface

class ControlInterface:

    def __init__(self):

        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.set_charging_srv = rospy.ServiceProxy(ros_node + model_name + '/set_charging', SetCharging)
        self.set_charge_rate_srv = rospy.ServiceProxy(ros_node + model_name + '/set_charge_rate', SetChargingRate)
        self.set_charge_srv = rospy.ServiceProxy(ros_node + model_name + '/set_charge', SetCharge)
        self.set_powerload_srv = rospy.ServiceProxy(ros_node + model_name + '/set_power_load', SetLoad)

        self.battery_charge = -1

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

    ci = ControlInterface()
    state = ci.get_bot_state()
    print("Bot is located at ({0}, {1}), facing {2} and going with a speed of {3} m/s".format(state[0], state[1], state[2], state[3]))

    # ci.set_power_load(1)
    ci.set_charge(3)
    ci.set_charging(0)

    monitor_battery()


if __name__ == '__main__':
    main()
