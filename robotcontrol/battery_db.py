import xml.etree.ElementTree as et


class BatteryDB:

    def __init__(self, world_file, battery_name):
        tree = et.parse(world_file)
        self.battery_voltage = 0
        self.charge_rate = 0
        self.capacity = 0
        for battery in tree.iter('battery'):
            if battery.get('name') == battery_name:
                self.battery_voltage = float(battery.find('voltage').text)
        # assuming the plugin name is "battery", see the world xml file structure where the battery plugin live
        for plugin in tree.iter('plugin'):
            if plugin.get('name') == 'battery':
                self.charge_rate = float(plugin.find('charge_rate').text)
                self.capacity = float(plugin.find('capacity').text)

    def time_to_fully_discharge(self, charge_level, power_load):
        """calculate Ah per second by dividing Power Load (in Watt) by Voltage to give the current"""
        """return the time in seconds"""
        draw_per_second = power_load / (self.battery_voltage * 3600)
        time_to_discharge = charge_level / draw_per_second
        return time_to_discharge

    def time_to_fully_charge(self, charge_level):
        """return in seconds"""
        charge_diff = self.capacity - charge_level
        recharge_time = (charge_diff / self.charge_rate) * 3600
        return recharge_time
