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
from core.target_vehicles_generation_protocols import *
import numpy as np
import csv

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")

from sumolib import checkBinary
from tqdm import tqdm
import traci


# use vehicle generation protocols to generate vehicle list
def get_controlled_vehicles(route_filename, connection_info, \
    num_controlled_vehicles=10, num_uncontrolled_vehicles=20, pattern = 1):
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
    print(connection_info.net_filename)
    generator = target_vehicles_generator(connection_info.net_filename)

    # list of target vehicles is returned by generate_vehicles
    vehicle_list = generator.generate_vehicles(num_controlled_vehicles, num_uncontrolled_vehicles, \
        pattern, route_filename, connection_info.net_filename)

    for vehicle in vehicle_list:
        vehicle_dict[str(vehicle.vehicle_id)] = vehicle

    return vehicle_dict

def test_dijkstra_policy(vehicles):
    print("Testing Dijkstra's Algorithm Route Controller")
    scheduler = DijkstraPolicy(init_connection_info)
    return run_simulation(scheduler, vehicles)

def test_density_dijkstra_policy(vehicles):
    print("Testing Test Route Controller")
    scheduler = DensityDijkstraPolicy(init_connection_info)
    return run_simulation(scheduler, vehicles)

def test_fw_policy(vehicles):
    print("Testing Floyd-Warshall Route Controller")
    scheduler = FloydWarshallPolicy(init_connection_info)
    return run_simulation(scheduler, vehicles)

def run_simulation(scheduler, vehicles):

    simulation = StrSumo(scheduler, init_connection_info, vehicles, csv_path)

    # add "--quit-on-end", "--start" to line below if you want to test
    # with sumo-gui while still skipping fast
    traci.start([sumo_binary, "--no-step-log", "-c", "./configurations/myconfig.sumocfg", \
                 "--tripinfo-output", "./configurations/trips.trips.xml", \
                 "--fcd-output", "./configurations/testTrace.xml"])

    total_time, end_number, deadlines_missed = simulation.run()
    # print("Average timespan: {}, total vehicle number: {}".format(str(total_time/end_number),\
    #     str(end_number)))
    # print(str(deadlines_missed) + ' deadlines missed.')
    traci.close()
    end_number = max(end_number, 1)
    return np.array([(total_time/end_number), end_number, deadlines_missed])


if __name__ == "__main__":

    for i in range(10000):
        print(i)

        csv_path = '/home/jack/Code/PycharmProjects/Selfless-Traffic-Routing-Testbed/testing_data/vehicle_results.csv'
        f = open(csv_path, "w+")
        writer = csv.writer(f)
        writer.writerow(['ControllerID', 'EndNumber', 'VehicleID', 'Timespan', 'DeadlineMissed'])
        f.close()

        # sumo_binary = checkBinary('sumo-gui')
        sumo_binary = checkBinary('sumo') #use this line if you do not want the UI of SUMO

        # parse config file for map file name
        dom = parse("./configurations/myconfig.sumocfg")

        net_file_node = dom.getElementsByTagName('net-file')
        net_file_attr = net_file_node[0].attributes

        net_file = net_file_attr['value'].nodeValue
        init_connection_info = ConnectionInfo("./configurations/"+net_file)

        route_file_node = dom.getElementsByTagName('route-files')
        route_file_attr = route_file_node[0].attributes
        route_file = "./configurations/"+route_file_attr['value'].nodeValue
        # first is num controlled, second is num uncontrolled
        vehicles = get_controlled_vehicles(route_file, init_connection_info, 150, 50)
        #print the controlled vehicles generated
        # for vid, v in vehicles.items():
        #     print("id: {}, destination: {}, start time:{}, deadline: {};".format(vid, \
        #         v.destination, v.start_time, v.deadline))
        times_one = test_dijkstra_policy(vehicles)
        times_two = test_fw_policy(vehicles)
        print(times_one)
        print(times_two)

        timespan_dijk = np.array(times_one)[0]
        print(f">> Dijkstra >> Average timespan: {timespan_dijk}, "
              f"deadlines missed: {np.array(times_one)[2]}")

        timespan_test = np.array(times_two)[0]
        print(f">> Test >> Average timespan: {timespan_test}, "
              f"deadlines missed: {np.array(times_two)[2]}")

        print(int(times_one[1]) == int(times_two[1]))
        print(timespan_dijk, timespan_test)
        if times_two[2] < 100:
            print("pass test 0")
            if abs(times_one[1]) - int(times_two[1]) < 10:
                print("pass test 1")
                if timespan_dijk > 100 and timespan_test > 100:
                    print("pass test 2")
                    if 1.5 < (timespan_dijk / timespan_test) < 2:
                        print("pass test 3")
                        break
