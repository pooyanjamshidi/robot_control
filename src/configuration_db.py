import json


class ConfigurationDB:

    def __init__(self, conf_db):
        with open(conf_db) as db:
            data = json.load(db)
        self.db = data['configurations']

    def get_power_load(self, conf_id):
        config = filter(lambda conf: conf['config_id'] == conf_id, self.db)
        return config[0]['power_load_w']

    def get_speed(self, conf_id):
        config = filter(lambda conf: conf['config_id'] == conf_id, self.db)
        return config[0]['speed']
