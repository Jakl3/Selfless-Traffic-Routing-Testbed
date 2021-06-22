from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy


class FloydWarshallPolicy(RouteController):
    """
    Example class for a custom scheduling algorithm.
    Utilizes a random decision policy until vehicle destination is within reach,
    then targets the vehicle destination.
    """

    def __init__(self, connection_info):
        """
        Constructor for the class
        :variable edge_lane_speed_list: contains the maximum speed of every edge, as defined by SUMO
        """
        super().__init__(connection_info)
        self.edge_lane_speed_list = {}

    def trace_path(self, i, j, p):
        """
        Reconstructs the path from edge i to edge j using the predecessor matrix constructed during
        the calculations of the Floyd-Warshall algorithm
        :param i: edge i, the initial edge
        :param j: edge j, the final edge
        :param p: the predecessor matrix, where p[i][j] contains the predecessor from i to j
        :return: path: an array containing the reconstructed path from edge i to edge j
        """
        if p[i][j] == -1:
            return {}

        path = [i]
        while i != j:
            i = p[i][j]
            path.append(i)

        return path

    def create_lane_speed(self):
        """
        Constructs the variable edge_lane_speed_list, filling it with the maximum speed of each edge.
        """
        for edge in self.connection_info.edge_list:
            if self.edge_lane_speed_list:
                return

        lanes_list = traci.lane.getIDList()
        self.edge_lane_speed_list = {edge: [] for edge in self.connection_info.edge_list}
        for lane in lanes_list:
            edge = traci.lane.getEdgeID(lane)
            if edge in self.edge_lane_speed_list:
                self.edge_lane_speed_list[edge].append(traci.lane.getMaxSpeed(lane))

    def make_decisions(self, vehicles, connection_info):
        """
        A scheduling algorithm that uses the Floyd-Warshall algorithm with travel-time-based weights to schedule
        vehicles along certain routes. These routes will be sent to each vehicle for them to execute.
        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """
        self.create_lane_speed()

        # Variables used for the Floyd-Warshall algorithm
        inf = 1e8
        n = len(self.connection_info.edge_list)
        edge_idx = self.connection_info.edge_index_dict

        # Computes the weight of edges
        weight = {}
        for edge in self.connection_info.edge_list:
            max_speed = max(self.edge_lane_speed_list[edge])
            length = self.connection_info.edge_length_dict[edge]

            # The commented section below does the exact same thing as the uncommented section. TraCI and SUMO
            # already computes the average speed, so we can just use that instead of calculating it ourselves.

            # vehicles_on_edge = traci.edge.getLastStepVehicleIDs(edge)
            #
            # if len(vehicles_on_edge) == 0:
            #     speed = traci.edge.getLastStepMeanSpeed(edge)
            #     print(">>> ", speed)
            # else:
            #     total_velocity = 0
            #     for veh in vehicles_on_edge:
            #         total_velocity += traci.vehicle.getSpeed(veh)
            #     speed = total_velocity * (1/len(vehicles_on_edge))
            #     print(">>>> ", total_velocity, len(vehicles_on_edge), speed)
            # print(traci.edge.getLastStepMeanSpeed(edge), traci.edge.getLastStepVehicleNumber(edge))

            traci_spd = traci.edge.getLastStepMeanSpeed(edge)
            speed = traci_spd if traci_spd != 0 else max_speed
            weight[edge] = length / speed

        # Initializes the distance matrix, where distance[i][j] stores the shortest path from i to j
        distance = [[inf if i != j else 0 for i in range(n)] for j in range(n)]
        for edge in self.connection_info.edge_list:
            idx1 = edge_idx[edge]
            for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[edge].items():
                idx2 = edge_idx[outgoing_edge]
                distance[idx1][idx2] = weight[edge]

        # Initializes predecessor matrix, where p[i][j] stores the predecessor from i to j
        p = [[0 for i in range(n)] for j in range(n)]
        for i in range(n):
            for j in range(n):
                if distance[i][j] == inf:
                    p[i][j] = -1
                else:
                    p[i][j] = j

        # Floyd-Warshall Algorithm, computes the shortest path for all pairs of edges i and j
        for k in range(n):
            for i in range(n):
                for j in range(n):
                    if distance[i][k] == inf or distance[k][j] == inf:
                        continue

                    if distance[i][k] + distance[k][j] < distance[i][j]:
                        distance[i][j] = distance[i][k] + distance[k][j]
                        p[i][j] = p[i][k]

        local_targets = {}
        for vehicle in vehicles:
            # Defines variables for easier use in the following sections
            current_edge = vehicle.current_edge
            destination = vehicle.destination
            curr = edge_idx[current_edge]
            end = edge_idx[destination]

            # Traces the shortest path between the origin (curr) and the destination (end)
            path = self.trace_path(curr, end, p)
            edge_path = []
            for edge in path:
                for key, value in edge_idx.items():
                    if edge == value:
                        edge_path.append(key)

            # Uses the reconstructed path to make a decisions list that can be used to compute instructions
            decision_list = []
            for i in range(len(edge_path) - 1):
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[edge_path[i]].items():
                    if outgoing_edge == edge_path[i + 1]:
                        decision_list.append(direction)

            # Creates the local_targets for each vehicle
            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
        return local_targets
