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
        super().__init__(connection_info)
        self.edge_lane_speed_list = {}

    def trace_path(self, i, j, p):
        if p[i][j] == -1:
            return {}

        path = [i]
        while i != j:
            i = p[i][j]
            path.append(i)

        return path

    def create_lane_speed(self):
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
        A custom scheduling algorithm can be written in between the 'Your algo...' comments.
        -For each car in the vehicle batch, your algorithm should provide a list of future decisions.
        -Sometimes short paths result in the vehicle reaching its local TRACI destination before reaching its
         true global destination. In order to counteract this, ask for a list of decisions rather than just one.
        -This list of decisions is sent to a function that returns the 'closest viable target' edge
          reachable by the decisions - it is not the case that all decisions will always be consumed.
          As soon as there is enough distance between the current edge and the target edge, the compute_target_edge
          function will return.
        -The 'closest viable edge' is a local target that is used by TRACI to control vehicles
        -The closest viable edge should always be far enough away to ensure that the vehicle is not removed
          from the simulation by TRACI before the vehicle reaches its true destination

        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """
        self.create_lane_speed()
        # print(self.create_lane_speed())

        inf = 1e8
        n = len(self.connection_info.edge_list)
        edge_idx = self.connection_info.edge_index_dict

        # computes weight of edges
        weight = {}
        for edge in self.connection_info.edge_list:
            max_speed = max(self.edge_lane_speed_list[edge])

            length = self.connection_info.edge_length_dict[edge]
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

        # print("passed weight assign")

        # initializes distance matrix
        distance = [[inf if i != j else 0 for i in range(n)] for j in range(n)]
        for edge in self.connection_info.edge_list:
            idx1 = edge_idx[edge]
            for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[edge].items():
                idx2 = edge_idx[outgoing_edge]
                distance[idx1][idx2] = weight[edge]

        # initializes predecessor matrix
        p = [[0 for i in range(n)] for j in range(n)]
        for i in range(n):
            for j in range(n):
                if distance[i][j] == inf:
                    p[i][j] = -1
                else:
                    p[i][j] = j

        # Floyd-Warshall Algorithm
        for k in range(n):
            for i in range(n):
                for j in range(n):
                    if distance[i][k] == inf or distance[k][j] == inf:
                        continue

                    if distance[i][k] + distance[k][j] < distance[i][j]:
                        distance[i][j] = distance[i][k] + distance[k][j]
                        p[i][j] = p[i][k]

        # print(distance)

        local_targets = {}
        for vehicle in vehicles:
            # print("\n\n\n\n\n\n\n\n")
            current_edge = vehicle.current_edge
            destination = vehicle.destination
            curr = edge_idx[current_edge]
            end = edge_idx[destination]
            # print(distance[curr][end])

            # traces the shortest path between curr and end
            path = self.trace_path(curr, end, p)
            edge_path = []
            for edge in path:
                for key, value in edge_idx.items():
                    if edge == value:
                        edge_path.append(key)
            # print(vehicle.current_edge, vehicle.destination, edge_path)

            decision_list = []
            for i in range(len(edge_path) - 1):
                # print(edge_path[i], self.connection_info.outgoing_edges_dict[edge_path[i]])
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[edge_path[i]].items():
                    # print(">>>>>>>", outgoing_edge, edge_path[i+1])
                    if outgoing_edge == edge_path[i + 1]:
                        # print("good >>", outgoing_edge, edge_path[i+1])
                        decision_list.append(direction)

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
        return local_targets
