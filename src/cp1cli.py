import rospy
from gazebo_msgs.srv import *

# importing battery services
from brass_gazebo_battery.srv import *

from tf.transformations import euler_from_quaternion
import math


# Here we manage the world, bot, and control interface

class ControlInterface:

    def __init__(self):

        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.set_charging_srv = rospy.ServiceProxy('/mobile_base/set_charging', SetCharging)
        self.set_charge_srv = rospy.ServiceProxy('/mobile_base/set_charge', SetCharge)
        self.set_powerload_srv = rospy.ServiceProxy('/mobile_base/set_power_load', SetLoad)


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
        return self.set_charging_srv(charge)

    def set_power_load(self, load):
        return  self.set_powerload_srv(load)


def main():
    ci = ControlInterface()
    state = ci.get_bot_state()
    print(state)


if __name__ == '__main__':
    main()
