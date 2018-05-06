import json


class ConfigurationDB:

    def __init__(self, conf_db):
        with open(conf_db) as db:
            data = json.load(db)
        self.db = data['configurations']

    def get_power_load(self, conf_id):
        config = list(filter(lambda conf: conf['config_id'] == conf_id, self.db))
        return config[0]['power_load_w']

    def get_speed(self, conf_id):
        config = list(filter(lambda conf: conf['config_id'] == conf_id, self.db))
        return config[0]['speed']

    def get_default_config(self):
        return 0

    def get_a_conservative_config(self):
        """returns a configuration with minimum power consumption"""
        config = 0
        for conf in self.db:
            if conf['power_load_w'] < self.db[config]['power_load_w']:
                config = conf['config_id']

        return config

    def get_a_highest_speed_config(self):
        """returns the speediest configuration and perhaps the most power consuming"""
        config = 0
        for conf in self.db:
            if conf['speed'] > self.db[config]['speed']:
                config = conf['config_id']

        return config
