import json
from robotcontrol.constants import AdaptationLevel


class ReadyDB:
    def __init__(self, ready_db):
        with open(ready_db) as db:
            data = json.load(db)
        self.db = data

    def get_budget(self):
        return self.db["discharge-budget"]

    def get_baseline(self):
        if self.db["level"] == "a":
            return AdaptationLevel.BASELINE_A
        elif self.db["level"] == "b":
            return AdaptationLevel.BASELINE_B
        elif self.db["level"] == "c":
            return AdaptationLevel.BASELINE_C

    def get_power_model(self):
        return "model" + str(self.db["power-model"])

    def get_start_location(self):
        return self.db["start-loc"]

    def get_target_locations(self):
        return self.db["target-locs"]
