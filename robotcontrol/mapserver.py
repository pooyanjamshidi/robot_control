#! /usr/bin/env python

import json
import numpy as np
import math
import heapq
from operator import itemgetter
import random


def distance(loc1, loc2):
    return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)


class MapServer:

    def __init__(self, map_file):
        with open(map_file) as db:
            data = json.load(db)
        self.waypoint_list = data["map"]
        self.waypoint_idx = {}
        for i in range(len(self.waypoint_list)):
            self.waypoint_idx[self.waypoint_list[i]['node-id']] = i

        if 'stations' in data:
            self.stations = data["stations"]

        self.adj_matrix = self.get_adjacency_matrix()
        self.waypoints = self.get_waypoints()

    def waypoint_to_coords(self, waypoint_id):
        """ given a way point, produce its coordinates """
        if not self.is_waypoint(waypoint_id):
            raise KeyError('The specified waypointID does not exist')
        waypoint_list = self.get_waypoint(waypoint_id)
        waypoint = waypoint_list[0]
        return waypoint['coords']

    def coords_to_waypoint(self, loc):
        """ given a location, it returns the closest waypoint id """
        closest_waypoint = {}
        for waypoint in self.waypoints:
            waypoint_loc = self.waypoint_to_coords(waypoint)
            d = distance([loc['x'], loc['y']], [waypoint_loc['x'], waypoint_loc['y']])
            if closest_waypoint.__len__() == 0:
                closest_waypoint['dist'] = d
                closest_waypoint['id'] = waypoint
            elif d < closest_waypoint['dist']:
                closest_waypoint['dist'] = d
                closest_waypoint['id'] = waypoint
        return closest_waypoint

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

    def get_charging_stations(self):
        return self.stations

    def get_waypoint(self, waypoint_id):
        return list(filter(lambda waypoint: waypoint["node-id"] == waypoint_id, self.waypoint_list))

    def get_waypoints(self):
        return self.waypoint_idx.keys()

    def idx_to_waypoint(self, idx):
        for k, v in self.waypoint_idx.items():
            if v == idx:
                return k

    def dfs_paths(self, start, goal):
        """
        Depth first search for deriving the paths from start to goal
        :param adj:
        :param start:
        :param goal:
        :return:
        """
        stack = [(start, [start])]
        L = len(self.waypoint_list)
        while stack:
            (vertex, path) = stack.pop()
            next_nodes = []
            for i in range(L):
                next_waypoint = self.idx_to_waypoint(i)
                if self.adj_matrix[self.waypoint_idx[vertex], i] == 1 and next_waypoint not in path:
                    next_nodes.append(next_waypoint)
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

    def closest_charging_station(self, waypoint):
        """Returns the closest path to s charging station

        :param waypoint_id:
        :return:
        """
        shortest_path = []
        for station in self.stations:
            paths = self.dfs_paths(waypoint, station)
            for path in paths:
                if len(shortest_path) == 0:
                    shortest_path = path
                elif len(path) < len(shortest_path):
                    shortest_path = path

        return shortest_path

    def get_two_closest_waypoints(self, x, y):
        distances_to_locs = {}
        for waypoint in self.waypoints:
            loc = self.waypoint_to_coords(waypoint)
            d = distance([x, y], [loc['x'], loc['y']])
            distances_to_locs[waypoint] = d

        #  place two obstacles on the closes waypoints to the current location of the robot
        two_closest_locs = heapq.nsmallest(2, distances_to_locs.iteritems(), itemgetter(1))
        loc1 = self.waypoint_to_coords(two_closest_locs[0][0])
        loc2 = self.waypoint_to_coords(two_closest_locs[1][0])

        return loc1, loc2

    def get_random_waypoint(self):
        """ get a random waypoint which is not aq charging station"""
        L = len(self.waypoints)
        waypoint = self.waypoints[random.randint(0, L-1)]
        while waypoint in self.stations:
            waypoint = self.waypoints[random.randint(0, L-1)]
        return waypoint
