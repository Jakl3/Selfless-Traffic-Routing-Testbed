'''
This test file needs the following files:
STR_SUMO.py, RouteController.py, Util.py, test.net.xml, test.rou.xml, myconfig.sumocfg and corresponding SUMO libraries.
'''
from core.STR_SUMO import StrSumo
import os
import sys
from xml.dom.minidom import parse, parseString
from core.Util import *
from controller.RouteController import *
from controller.DijkstraController import DijkstraPolicy
from controller.DensityDijkstraController import DensityDijkstraPolicy
from controller.FloydWarshallController import FloydWarshallPolicy
from controller.HeuristicController import HeuristicPolicy
from core.target_vehicles_generation_protocols import *
import numpy as np
import csv
from collections import defaultdict

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")

from sumolib import checkBinary
import traci


# use vehicle generation protocols to generate vehicle list
def get_controlled_vehicles(route_filename, connection_info,
                            num_controlled_vehicles=10, num_uncontrolled_vehicles=20, pattern=1):
    '''
    :param @route_filename <str>: the name of the route file to generate
    :param @connection_info <object>: an object that includes the map inforamtion
    :param @num_controlled_vehicles <int>: the number of vehicles controlled by the route controller
    :param @num_uncontrolled_vehicles <int>: the number of vehicles not controlled by the route controller
    :param @pattern <int>: one of four possible patterns. FORMAT:
            -- CASES BEGIN --
                #1. one start point, one destination for all target vehicles
                #2. ranged start point, one destination for all target vehicles
                #3. ranged start points, ranged destination for all target vehicles
            -- CASES ENDS --
    '''
    vehicle_dict = {}
    generator = target_vehicles_generator(connection_info.net_filename)

    # list of target vehicles is returned by generate_vehicles
    vehicle_list = generator.generate_vehicles(num_controlled_vehicles, num_uncontrolled_vehicles,
                                               pattern, route_filename, connection_info.net_filename)

    for vehicle in vehicle_list:
        vehicle_dict[str(vehicle.vehicle_id)] = vehicle

    return vehicle_dict


def test_dijkstra_policy(vehicles):
    print("Testing Dijkstra's Algorithm Route Controller")
    scheduler = DijkstraPolicy(init_connection_info)
    return run_simulation(scheduler, vehicles)


def test_density_policy(vehicles):
    print("Testing Density Dijkstra's Algorithm Route Controller")
    scheduler = DensityDijkstraPolicy(init_connection_info)
    return run_simulation(scheduler, vehicles)


def test_fw_policy(vehicles):
    print("Testing Floyd-Warshall Route Controller")
    scheduler = FloydWarshallPolicy(init_connection_info)
    return run_simulation(scheduler, vehicles)


def test_astar_policy(vehicles):
    print("Testing A-Star Route Controller")
    scheduler = HeuristicPolicy(init_connection_info)
    return run_simulation(scheduler, vehicles)


def run_simulation(scheduler, vehicles):
    simulation = StrSumo(scheduler, init_connection_info, vehicles)

    # add "--quit-on-end", "--start" to line below if you want to test
    # with sumo-gui while still skipping fast
    traci.start([sumo_binary, "--quit-on-end", "--start", "--no-step-log", "-c", "./configurations/myconfig.sumocfg",
                 "--tripinfo-output", "./configurations/trips.trips.xml",
                 "--fcd-output", "./configurations/testTrace.xml"])

    total_time, end_number, deadlines_missed = simulation.run()
    traci.close()

    end_number = max(end_number, 1)

    print("Average timespan: {}, total vehicle number: {}".format(str(total_time / end_number),
                                                                  str(end_number)))
    print(str(deadlines_missed) + ' deadlines missed.')

    return np.array([(total_time / end_number), end_number, deadlines_missed])

if __name__ == "__main__":

    res = defaultdict(list)

    csv_path = '/home/jack/Code/PycharmProjects/Selfless-Traffic-Routing-Testbed/testing_data/csv_files/1000_set.csv'

    with open(csv_path, mode="w") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['ControllerID', 'Timespan', 'DeadlineMissed', 'NumOfCars'])

    for i in range(1000):
        print('>>>', i)

        # sumo_binary = checkBinary('sumo-gui')
        sumo_binary = checkBinary('sumo')  # use this line if you do not want the UI of SUMO

        # parse config file for map file name
        dom = parse("./configurations/myconfig.sumocfg")

        net_file_node = dom.getElementsByTagName('net-file')
        net_file_attr = net_file_node[0].attributes

        net_file = net_file_attr['value'].nodeValue
        init_connection_info = ConnectionInfo("./configurations/" + net_file)

        route_file_node = dom.getElementsByTagName('route-files')
        route_file_attr = route_file_node[0].attributes
        route_file = "./configurations/" + route_file_attr['value'].nodeValue
        # first is num controlled, second is num uncontrolled
        vehis = get_controlled_vehicles(route_file, init_connection_info,
                                           num_controlled_vehicles=150,
                                           num_uncontrolled_vehicles=50,
                                           pattern=1)
        # # print the controlled vehicles generated
        # for vid, v in vehis.items():
        #     print("id: {}, destination: {}, start time:{}, deadline: {};".format(vid,
        #                                                                          v.destination, v.start_time,
        #                                                                          v.deadline))
        times = dict()
        times['astar'] = test_astar_policy(vehis)
        times['dijk'] = test_dijkstra_policy(vehis)

        for key in times:
            print(f">> {key} >> Average timespan: {np.array(times[key])[0]:.3f}, "
                  f"deadlines missed: {int(np.array(times[key])[2])}/{int(np.array(times[key])[1])}")
            res[key].append(times[key])
            with open(csv_path, mode="a") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([key,  # Controller ID
                                 np.array(times[key])[0],  # Timespan
                                 int(np.array(times[key])[2]),  # DeadlineMissed
                                 int(np.array(times[key])[1])])  # NumOfCars
        print()

    print('\n' * 5)

    for key in res:
        times = []
        deadlines = []
        tot = []

        for item in res[key]:
            times.append(item[0])
            tot.append(item[1])
            deadlines.append(item[2])

        timespan = np.mean(times)
        tot_deadlines = np.sum(deadlines)
        tot_num_of_cars = np.sum(tot)
        avg_deadlines = np.mean(deadlines)
        avg_num_of_cars = np.mean(tot)
        print(f">> {key} >> Average timespan: {timespan}, "
              f"Total deadlines missed: {tot_deadlines}/{tot_num_of_cars}, "
              f"Average deadlines missed: {avg_deadlines}/{avg_num_of_cars}")

    print('\n' * 5)
