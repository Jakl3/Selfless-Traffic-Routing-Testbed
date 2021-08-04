from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy
import heapq
from collections import defaultdict


class PriorityQueue:
    """
    Standard PriorityQueue data structure, adapted from online resources.
    """

    def __init__(self):
        self.elements = []

    def empty(self) -> bool:
        return not self.elements

    def put(self, item, priority: float):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


class HeuristicPolicy(RouteController):

    def __init__(self, connection_info):
        super().__init__(connection_info)
        self.edge_lane_speed_list = {}
        self.edge_idx = self.connection_info.edge_index_dict
        self.weight = None
        self.distance = None
        self.saved_vehicle_length = 5

    def create_lane_speed(self):
        """
        Constructs the variable edge_lane_speed_list, filling it with the maximum speed of each edge.
        """
        lanes_list = traci.lane.getIDList()
        self.edge_lane_speed_list = {edge: 0 for edge in self.connection_info.edge_list}
        for lane in lanes_list:
            edge = traci.lane.getEdgeID(lane)
            if edge in self.edge_lane_speed_list:
                self.edge_lane_speed_list[edge] = max(traci.lane.getMaxSpeed(lane),
                                                      self.edge_lane_speed_list[edge])

    def generate_floyd_warshall(self):
        """
        Generates the distance for Floyd-Warshall to use as heuristic.
        """
        # Variables used for the Floyd-Warshall algorithm
        inf = 1e9
        n = len(self.connection_info.edge_list)

        # Initializes the distance matrix, where distance[i][j] stores the shortest path from i to j
        distance = [[inf if i != j else 0 for i in range(n)] for j in range(n)]
        for edge in self.connection_info.edge_list:
            idx1 = self.edge_idx[edge]
            for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[edge].items():
                idx2 = self.edge_idx[outgoing_edge]
                distance[idx1][idx2] = self.weight[edge]

        # Computes the shortest path for all pairs of edges i and j
        for k in range(n):
            for i in range(n):
                for j in range(n):
                    if distance[i][k] == inf or distance[k][j] == inf:
                        continue
                    if distance[i][k] + distance[k][j] < distance[i][j]:
                        distance[i][j] = distance[i][k] + distance[k][j]

        return distance

    def compute_weights(self):
        """
        Computes the weights of the edges based on the estimated travel time
        across that edge. The estimate is based on a log of the ratio of maximum
        occupancy to current occupancy of the edge.
        """
        weight = {}
        for edge in self.connection_info.edge_list:
            max_speed = self.edge_lane_speed_list[edge]  # maximum speed on the edge
            length = self.connection_info.edge_length_dict[edge]  # length of the edge

            vehicle_number = traci.edge.getLastStepVehicleNumber(edge)  # number of vehicles on the edge
            vehicle_number = max(vehicle_number, 0.01)  # account for case where there are no vehicles on edge

            vehicle_length = max(traci.edge.getLastStepLength(edge),  # average length of vehicles on the edge
                                 self.saved_vehicle_length)  # if there are no vehicles, use this saved length
            max_cars = length / (1.3 * vehicle_length)  # multiplied to account for space between vehicles
            max_cars = max(vehicle_number, max_cars)  # max_cars might not be fully accurate, this fixes it

            if max_cars == vehicle_number:  # accounts for case where edge is full
                max_cars += 1

            speed = max_speed * np.log(max_cars / vehicle_number)  # function to estimate speed

            weight[edge] = length / speed  # estimates travel time and assigns its
        return weight

    def make_decisions(self, vehicles, connection_info):
        """
        make_decisions uses a modified A-star Algorithm to find the shortest path between each individual
        vehicle's current position and its destination. The heuristic used in the A-star algorithm if the
        remaining travel time between its current position and its destination, where a lower travel time
        is better. This remaining travel time is calculated using the Floyd-Warshall algorithm.
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        if len(self.edge_lane_speed_list) == 0:  # If the max_speed list isn't generated yet, generate it
            self.create_lane_speed()

        self.weight = self.compute_weights()  # Calculate the weights based on travel time

        # Calculate the distances between every node using Floyd-Warshall
        self.distance = self.generate_floyd_warshall()

        local_targets = {}  # this will store the targets for each vehicle

        sorted_vehicles = sorted(vehicles, key=lambda x: x.start_time)  # Sort the vehicle set by start time.

        # Iteratively loop through each vehicle
        for vehicle in sorted_vehicles:
            # Parents stores the path from current edge to destination.
            parents = self.a_star_search(vehicle.current_edge, vehicle.destination, vehicle.deadline)

            # Helper function to reconstruct the path of the A* algorithm.
            def reconstruct_path(previous, start, goal):
                current = goal
                path = []
                while current != start:
                    path.append(current)
                    current = previous[current]
                path.append(start)
                path.reverse()
                return path
            path_list = reconstruct_path(parents, vehicle.current_edge, vehicle.destination)

            # Uses the path list to create a set of instructions (in the form of directions) for vehicle
            decision_list = list()
            for i in range(len(path_list) - 1):
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[path_list[i]].items():
                    if outgoing_edge == path_list[i + 1]:
                        decision_list.append(direction)
                        continue

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
        return local_targets

    def heuristic(self, a, b, deadline, mu=0, sigma=50):
        """
        The heuristic is based on the previously calculated estimated total travel time from the
        Floyd-Warshall algorithm, and the deadline of the vehicle.
        :param a: the current edge
        :param b: the goal edge
        :param deadline: the deadline of the vehicle
        :param mu: mean, used for z score
        :param sigma: standard deviation, used for z score
        """
        tt = self.distance[self.edge_idx[a]][self.edge_idx[b]]
        # This allows for negatives: ie. if the estimated travel time of this path passes the allowed
        # deadline, the heuristic is positive and the cost for the edge is increased. Else, the heuristic
        # is negative and the cost decreases, thereby encouraging the algorithm to pick this edge.
        remaining_tt = (tt + traci.simulation.getTime()) - deadline
        # print(a, b, tt, remaining_tt, traci.simulation.getTime(), deadline)
        z = (remaining_tt - mu) / sigma  # uses the Z-score, since the remaining travel time can be very large,
                                         # leading to a large skew in how the weight is considered
        return z

    def a_star_search(self, start, goal, deadline):
        """
        Performs search using the A* algorithm with a custom heuristic. Finds shortest path
        from the vehicle's current edge to its destination, trying to stay within the deadline.
        :param start: the current edge/starting point of the vehicle
        :param goal: the destination of the vehicle
        :param  deadline: the deadline of the vehicle
        :return parents: the shortest path from start to goal
        """
        front = PriorityQueue()
        front.put(start, 0)

        parents = dict()
        costs = dict()
        parents[start] = None
        costs[start] = 0

        while not front.empty():
            current = front.get()
            if current == goal:  # found destination
                break
            for _, outgoing_edge in self.connection_info.outgoing_edges_dict[current].items():
                new_cost = costs[current] + self.weight[outgoing_edge]  # new costs is current plus new travel time
                if outgoing_edge not in costs or new_cost < costs[outgoing_edge]:
                    costs[outgoing_edge] = new_cost
                    heuristic = self.heuristic(outgoing_edge, goal, deadline)
                    weighted_cost = new_cost + heuristic
                    front.put(outgoing_edge, weighted_cost)
                    parents[outgoing_edge] = current

        return parents
