#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

import glob
import os
import sys


try:
    sys.path.append("PythonAPI/carla-0.9.6-py3.5-linux-x86_64.egg")
except IndexError:
    pass

import carla

import argparse
import random
import time
import csv
import numpy as np

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=10,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-d', '--delay',
        metavar='D',
        default=2.0,
        type=float,
        help='delay in seconds between spawns (default: 2.0)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    args = argparser.parse_args()

    actor_list = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)

    try:

        world = client.get_world()
        blueprints = world.get_blueprint_library()
        bp_vehicles = blueprints.filter('vehicle.*')
        bp_pedestrians = blueprints.filter('walker.*')

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            
        
        def try_spawn_random_vehicle_at(transform):
            blueprint = random.choice(bp_vehicles)

            if(int(blueprint.get_attribute('number_of_wheels')) == 4):
                blueprint = bp_vehicles.filter('vehicle.bmw.*')[0]
            elif(int(blueprint.get_attribute('number_of_wheels')) == 2):
                blueprint = bp_vehicles.filter('vehicle.harley-davidson.*')[0]
        
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            blueprint.set_attribute('role_name', 'autopilot')
            vehicle = world.try_spawn_actor(blueprint, transform)
            if vehicle is not None:
                actor_list.append(vehicle)
                vehicle.set_autopilot()
#                vehicle.apply_control(carla.VehicleControl(throttle=0.05))
#                print('spawned %r at %s' % (vehicle.type_id, transform.location))
                return True
            return False

        # @todo Needs to be converted to list to be shuffled.
        spawn_points = list(world.get_map().get_spawn_points())
#        random.shuffle(spawn_points)
#
#        print('found %d spawn points.' % len(spawn_points))

        count = args.number_of_vehicles
        other_cars = []
        
        spawn_array = np.genfromtxt('other_vehicle_spawn_points.csv', delimiter=',')
        spawn_old=[]
        for i in range(len(spawn_array)):
            spawn_old.append(spawn_array[i])
        spawn_old = np.asarray(spawn_old)
            
            
        flag = 0
        for spawn_point in spawn_points:
#            other_cars.append([spawn_point.location.x,spawn_point.location.y,spawn_point.location.z, 
#                               spawn_point.rotation.pitch,spawn_point.rotation.yaw,spawn_point.rotation.roll])
            spawn_point.location.x = spawn_old[flag][0]
            spawn_point.location.y = spawn_old[flag][1]
            spawn_point.location.z = spawn_old[flag][2]
            spawn_point.rotation.pitch = spawn_old[flag][3]
            spawn_point.rotation.yaw = spawn_old[flag][4]
            spawn_point.rotation.roll = spawn_old[flag][5]
            
            if try_spawn_random_vehicle_at(spawn_point):
                flag = flag + 1 
                count -= 1
            if count <= 0:
                break
            
        other_cars = np.asarray(other_cars)

        while count > 0:
            time.sleep(args.delay)
            if try_spawn_random_vehicle_at(random.choice(spawn_points)):
                count -= 1

        print('spawned %d vehicles, press Ctrl+C to exit.' % args.number_of_vehicles)

        while True:
            time.sleep(10)

    finally:
#        np.savetxt("other_vehicle_spawn_points.csv", other_cars, delimiter=",")
            
        print('\ndestroying %d actors' % len(actor_list))
        client.apply_batch([carla.command.DestroyActor(x.id) for x in actor_list])


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
