from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy


class DensityDijkstraPolicy(RouteController):

    def __init__(self, connection_info):
        super().__init__(connection_info)

    def make_decisions(self, vehicles, connection_info):
        """
        A scheduling algorithm that uses Dijkstra's Algorithm to find the shortest path to each
        individual vehicle's destination. The weights for each edge are weighted by the density of the
        length of that edge.
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        local_targets = {}
        for vehicle in vehicles:
            decision_list = []
            unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list}  # map of unvisited edges
            visited = {}  # map of visited edges
            current_edge = vehicle.current_edge

            current_distance = self.connection_info.edge_length_dict[current_edge]
            unvisited[current_edge] = current_distance
            path_lists = {edge: [] for edge in
                          self.connection_info.edge_list}  # stores shortest path to each edge using directions
            while True:

                # Creates new length dictionary based on the density of the edge
                len_dict = {}
                for edge_now in self.connection_info.edge_list:
                    car_num = traci.edge.getLastStepVehicleNumber(edge_now)
                    density = car_num / self.connection_info.edge_length_dict[edge_now]
                    len_dict[edge_now] = max((self.connection_info.edge_length_dict[edge_now]),
                                             (self.connection_info.edge_length_dict[edge_now]) * (100*density))

                if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                    continue
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                    if outgoing_edge not in unvisited:
                        continue
                    new_distance = current_distance + len_dict[outgoing_edge]
                    new_distance_2 = current_distance + self.connection_info.edge_length_dict[outgoing_edge]
                    if new_distance < unvisited[outgoing_edge]:
                        # The edge dictionary used for the next step is the set as the original length, not the
                        # weighted length of that edge
                        unvisited[outgoing_edge] = new_distance_2
                        current_path = copy.deepcopy(path_lists[current_edge])
                        current_path.append(direction)
                        path_lists[outgoing_edge] = copy.deepcopy(current_path)

                visited[current_edge] = current_distance
                del unvisited[current_edge]
                if not unvisited:
                    break
                if current_edge == vehicle.destination:
                    break
                possible_edges = [edge for edge in unvisited.items() if edge[1]]
                current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]

            for direction in path_lists[vehicle.destination]:
                decision_list.append(direction)

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
        return local_targets

