import json
import numpy as np


class MapServer:

    def __init__(self, map_file):
        with open(map_file) as map:
            data = json.load(map)
            self.waypoint_list = data["map"]
            self.waypoint_idx = {}
            for i in range(len(self.waypoint_list)):
                self.waypoint_idx[self.waypoint_list[i]] = i

            if 'stations' in data:
                self.stations = data["stations"]

            self.adj_matrix = self.get_adjacency_matrix()

    def waypoint_to_coords(self, waypoint_id):
        """ given a way point, produce its coordinates """
        if not self.is_waypoint(waypoint_id):
            raise KeyError('The specified waypointID does not exist')
        waypoint_list = self.get_waypoint(waypoint_id)
        waypoint = waypoint_list[0]
        return waypoint['coords']

    def is_waypoint(self, waypoint_id):
        """ given a string, determine if it is actually a waypoint id """
        waypoint_list = self.get_waypoint(waypoint_id)
        if len(waypoint_list) > 1:
            raise ValueError('non-unique waypoint identifiers in the map file')
        return len(waypoint_list) == 1

    def is_charging_station(self, waypoint_id):
        if waypoint_id in self.stations:
            return True
        else:
            return False

    def get_charging_station(self):
        return self.stations

    def get_waypoint(self, waypoint_id):
        return filter(lambda waypoint: waypoint["node-id"] == waypoint_id, self.waypoint_list)

    def get_waypoints(self):
        return self.waypoint_list

    def dfs_paths(self, adj, start, goal):
        """
        Depth first search for deriving the paths from start to goal
        :param adj:
        :param start:
        :param goal:
        :return:
        """
        stack = [(start, [start])]
        while stack:
            (vertex, path) = stack.pop()
            next_nodes = []
            for i in range(adj[vertex, :].size):
                if adj[vertex, i] == 1 and i not in path:
                    next_nodes.append(i)
            for next in next_nodes:
                if next == goal:
                    yield path + [next]
                else:
                    stack.append((next, path + [next]))

    def get_adjacency_matrix(self):
        """Transform the json to adjacency matrix.

        :return:
        """
        L = len(self.waypoint_list)
        adj = np.zeros((L, L), dtype=int)

        for i in range(L):
            waypoint_id = self.waypoint_list[i]["node-id"]
            waypoint_idx = self.waypoint_idx[waypoint_id]
            connected_to = self.waypoint_list[i]["connected-to"]
            for j in range(len(connected_to)):
                waypoint_connected_idx = self.waypoint_idx[connected_to[j]]
                adj[waypoint_idx, waypoint_connected_idx] = 1

        return adj

    def closest_charging_station(self, waypoint_id):
        """Returns the closest path to s charging station

        :param waypoint_id:
        :return:
        """
        shortest_path = []
        for station in self.stations:
            paths = self.dfs_paths(waypoint_id, station)
            for path in paths:
                if len(shortest_path) > 0 and len(path) < len(shortest_path):
                    shortest_path = path

        return shortest_path
