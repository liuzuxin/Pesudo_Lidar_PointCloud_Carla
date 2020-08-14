#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  7 16:02:32 2019

@author: akhildb
"""

#!/usr/bin/env python

# Copyright (c) 2019 Aptiv
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import json

try:
    sys.path.append("PythonAPI/carla-0.9.6-py3.5-linux-x86_64.egg")
except IndexError:
    pass

# ==============================================================================
# -- add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('PythonAPI')[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc
from agents.navigation.roaming_agent import *
from agents.navigation.basic_agent import *

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla

import weakref
import random

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import time 

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

import tqdm

# VIEW_WIDTH = 1920//2
# VIEW_HEIGHT = 1080//2
VIEW_WIDTH = 533
VIEW_HEIGHT = 400
VIEW_FOV = 90

BB_COLOR = (248, 64, 24)

WINDOW_WIDTH = 533
WINDOW_HEIGHT = 300
MINI_WINDOW_WIDTH = 533
MINI_WINDOW_HEIGHT = 400
number_lidar = 2
ROI = 50

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate-1] + u'\u2026') if len(name) > truncate else name


class World(object):
    def __init__(self, carla_world, width, height, current_car, gap_x, displace_y,delta_seconds,config,lidar_number):
        self.world = carla_world
        self.width = width
        self.height = height
        self.map = self.world.get_map()
        self.delta_seconds = delta_seconds
        self.config = config
        self.lidar_number = lidar_number
        # self.hud = hud
        if current_car is not None:
            self.player = current_car
        else:
            self.player = None 
        # self.collision_sensor = None
        # self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = [None] * self.lidar_number
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        # self._actor_filter = actor_filter
        self.recording_enabled = False
        self.recording_start = 0
        self.surface_gap_x = gap_x
        self.surface_diplace_y = displace_y
        self.restart()  
        # self.world.on_tick(hud.on_world_tick)


    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = []
        cam_pos_index = []
        for i in range(self.lidar_number):
            cam_index.append(self.camera_manager[i]._index if self.camera_manager[i] is not None else 0)
            cam_pos_index.append(self.camera_manager[i]._transform_index if self.camera_manager[i] is not None else 0)
        
        
        blueprint = self.world.get_blueprint_library().find('vehicle.lincoln.mkz2017')
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        while self.player is None:
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        
        for i in range(self.lidar_number):
            self.camera_manager[i] = CameraManager(self.player, self.width, self.height, self.surface_gap_x, self.surface_diplace_y,
                                            [self.config[6*i],self.config[6*i+1],self.config[6*i+2],self.config[6*i+3],
                                             self.config[6*i+4],self.config[6*i+5]],i)
            self.camera_manager[i]._transform_index = cam_pos_index[i]
            self.camera_manager[i].set_sensor(cam_index[i], notify=False)
        
        print("Camera manager(s) initialized")
        print("set image")
        print("got player")


    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        #self.world.on_tick()
        self.world.tick()
        pass

    def render(self, display):
        self.camera_manager[0].render(display)
        # self.hud.render(display)

    def destroySensors(self):
        for i in range(self.lidar_number):
            self.camera_manager[i].sensor.destroy()
            self.camera_manager[i].sensor = None
            self.camera_manager[i] = None
        

    def destroy(self):
        for i in range(self.lidar_number):
            actors = [self.camera_manager[i].sensor,self.player]
            for actor in actors:
                if actor is not None:
                    actor.destroy()
                

#class KeyboardControl(object):
#    def __init__(self, world, start_in_autopilot):
#        self._autopilot_enabled = start_in_autopilot
#        if isinstance(world.player, carla.Vehicle):
#            self._control = carla.VehicleControl()
#            world.player.set_autopilot(self._autopilot_enabled)
#        elif isinstance(world.player, carla.Walker):
#            self._control = carla.WalkerControl()
#            self._autopilot_enabled = False
#            self._rotation = world.player.get_transform().rotation
#        else:
#            raise NotImplementedError("Actor type not supported")
#        self._steer_cache = 0.0
#
#    def parse_events(self, client, world, clock):
#        for event in pygame.event.get():
#            if event.type == pygame.QUIT:
#                return True
#            elif event.type == pygame.KEYUP:
#                if self._is_quit_shortcut(event.key):
#                    return True
#                elif event.key == K_BACKSPACE:
#                    world.restart()
#                elif event.key == K_F1:
#                    # world.hud.toggle_info()
#                    pass
#                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
#                    # world.hud.help.toggle()
#                    pass
#                elif event.key == K_TAB:
#                    world.camera_manager.toggle_camera()
#                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
#                    world.next_weather(reverse=True)
#                elif event.key == K_c:
#                    world.next_weather()
#                elif event.key == K_BACKQUOTE:
#                    world.camera_manager.next_sensor()
#                elif event.key > K_0 and event.key <= K_9:
#                    world.camera_manager.set_sensor(event.key - 1 - K_0)
#                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
#                    world.camera_manager.toggle_recording()
#                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
#                    if (world.recording_enabled):
#                        client.stop_recorder()
#                        world.recording_enabled = False
#                        world.hud.notification("Recorder is OFF")
#                    else:
#                        client.start_recorder("manual_recording.rec")
#                        world.recording_enabled = True
#                        world.hud.notification("Recorder is ON")
#                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
#                    # stop recorder
#                    client.stop_recorder()
#                    world.recording_enabled = False
#                    # work around to fix camera at start of replaying
#                    currentIndex = world.camera_manager._index
#                    world.destroySensors()
#                    # disable autopilot
#                    self._autopilot_enabled = False
#                    world.player.set_autopilot(self._autopilot_enabled)
#                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
#                    world.camera_manager.set_sensor(currentIndex)
#                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
#                    if pygame.key.get_mods() & KMOD_SHIFT:
#                        world.recording_start -= 10
#                    else:
#                        world.recording_start -= 1
#                    # world.hud.notification("Recording start time is %d" % (world.recording_start))
#                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
#                    if pygame.key.get_mods() & KMOD_SHIFT:
#                        world.recording_start += 10
#                    else:
#                        world.recording_start += 1
#                    # world.hud.notification("Recording start time is %d" % (world.recording_start))
#                if isinstance(self._control, carla.VehicleControl):
#                    if event.key == K_q:
#                        self._control.gear = 1 if self._control.reverse else -1
#                    elif event.key == K_m:
#                        self._control.manual_gear_shift = not self._control.manual_gear_shift
#                        self._control.gear = world.player.get_control().gear
#                        # world.hud.notification('%s Transmission' % ('Manual' if self._control.manual_gear_shift else 'Automatic'))
#                    elif self._control.manual_gear_shift and event.key == K_COMMA:
#                        self._control.gear = max(-1, self._control.gear - 1)
#                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
#                        self._control.gear = self._control.gear + 1
#                    elif event.key == K_p and not (pygame.key.get_mods() & KMOD_CTRL):
#                        self._autopilot_enabled = not self._autopilot_enabled
#                        world.player.set_autopilot(self._autopilot_enabled)
#                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
#        if not self._autopilot_enabled:
#            if isinstance(self._control, carla.VehicleControl):
#                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
#                self._control.reverse = self._control.gear < 0
#            elif isinstance(self._control, carla.WalkerControl):
#                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
#            world.player.apply_control(self._control)
#
#    def _parse_vehicle_keys(self, keys, milliseconds):
#        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
#        steer_increment = 5e-4 * milliseconds
#        if keys[K_LEFT] or keys[K_a]:
#            self._steer_cache -= steer_increment
#        elif keys[K_RIGHT] or keys[K_d]:
#            self._steer_cache += steer_increment
#        else:
#            self._steer_cache = 0.0
#        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
#        self._control.steer = round(self._steer_cache, 1)
#        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
#        self._control.hand_brake = keys[K_SPACE]
#
#    def _parse_walker_keys(self, keys, milliseconds):
#        self._control.speed = 0.0
#        if keys[K_DOWN] or keys[K_s]:
#            self._control.speed = 0.0
#        if keys[K_LEFT] or keys[K_a]:
#            self._control.speed = .01
#            self._rotation.yaw -= 0.08 * milliseconds
#        if keys[K_RIGHT] or keys[K_d]:
#            self._control.speed = .01
#            self._rotation.yaw += 0.08 * milliseconds
#        if keys[K_UP] or keys[K_w]:
#            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
#        self._control.jump = keys[K_SPACE]
#        self._rotation.yaw = round(self._rotation.yaw, 1)
#        self._control.direction = self._rotation.get_forward_vector()
#
#    @staticmethod
#    def _is_quit_shortcut(key):
#        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


class CameraManager(object):
    def __init__(self, parent_actor, width, height, gap_x, diplace_y, config, lidar_number):
        self.save_counter = [0,0]
        self.width = width
        self.height = height
        self.sensor = None
        self._surface = None
        self._parent = parent_actor
        # self._hud = hud
        self._recording = False
        self.x = config[0]
        self.y = config[1]
        self.z = config[2]
        self.yaw = config[3]
        self.pitch = config[4]
        self.roll = config[5]
        self.lidar_number = lidar_number
        self.timedata = time.strftime("%Y-%d-%I-%M-%S",time.localtime(time.time()))
        timedata = self.timedata
        self.dirpath = "{0}_datacollected_{1}".format(self.lidar_number,timedata)
        self.cur_dir = os.path.dirname(os.path.realpath(__file__))
        self.save_dir = self.cur_dir +"/"+self.dirpath
        
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self._camera_transforms = [
            carla.Transform(carla.Location(x=self.x,y=self.y,z=self.z),carla.Rotation(yaw=config[3],pitch=config[4],roll=config[5])),
            carla.Transform(carla.Location(x=self.x,y=self.y,z=self.z),carla.Rotation(yaw=90+config[3],pitch=config[4],roll=config[5])),
            carla.Transform(carla.Location(x=self.x,y=self.y,z=self.z),carla.Rotation(yaw=180+config[3],pitch=config[4],roll=config[5])),
            carla.Transform(carla.Location(x=self.x,y=self.y,z=self.z),carla.Rotation(yaw=270+config[3],pitch=config[4],roll=config[5])),
            carla.Transform(carla.Location(x=self.x,y=self.y,z=self.z),carla.Rotation(yaw=config[3],pitch=config[4],roll=config[5]))]
        self._transform_index = 0
        self._sensors = [
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']
            ]
        self.fov = 90.0
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        print("setting image")
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
#                bp.set_attribute('image_size_x', str(self.width))
#                bp.set_attribute('image_size_y', str(self.height))
                bp.set_attribute('fov', str(self.fov))
                bp.set_attribute('image_size_x', '500')
                bp.set_attribute('image_size_y', '420') 
                print("set image six")
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '5000')
                bp.set_attribute('rotation_frequency', '1200')
                bp.set_attribute('upper_fov', '15')
                bp.set_attribute('lower_fov', '-25')
                bp.set_attribute('channels', '32')
                bp.set_attribute('points_per_second', '1000000')
            item.append(bp)

        self.surface_shift_x = gap_x
        self.surface_shift_y = diplace_y
        self.lidar_points = None
        self.depth_image = None

    def toggle_camera(self):
        self._transform_index = (self._transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self._sensors)
        print(index)
        print("making cameras")
        self._index = index
        if 1:
            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            print("sensor spawn1")
            self.sensor1 = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index+1],
                attach_to=self._parent)
            print("sensor spawn2")
            self.sensor2 = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index+2],
                attach_to=self._parent)
            print("sensor spawn3")
            self.sensor3 = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index+3],
                attach_to=self._parent)
            print("sensor spawn4")
            self.sensor4 = self._parent.get_world().spawn_actor(
                self._sensors[index+1][-1],
                self._camera_transforms[self._transform_index+4],
                attach_to=self._parent)
            print("sensor spawn5")
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image, "head"))
            self.sensor1.listen(lambda image1: CameraManager._parse_image(weak_self, image1 , "left"))
            self.sensor2.listen(lambda image2: CameraManager._parse_image(weak_self, image2, "tail"))
            self.sensor3.listen(lambda image3: CameraManager._parse_image(weak_self, image3, "right"))
            self.sensor4.listen(lambda image4: CameraManager._parse_image(weak_self, image4, "lidar"))
        if notify:
            # self._hud.notification(self._sensors[index][2])
            pass
        

    def next_sensor(self):
        self.set_sensor(self._index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        # self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        if self._surface is not None:
            display.blit(self._surface, (self.surface_shift_x, self.surface_shift_y))

    def save_lidar_to_file(self,lidar_num):
        file_name = "{0:06d}".format(self.save_counter[lidar_num-1])
        try:
            self.head_image.save_to_disk("{0}_datacollected_images_{1}/head/{2}".format(lidar_num, self.timedata, self.save_counter[lidar_num-1]))
            self.left_image.save_to_disk("{0}_datacollected_images_{1}/left/{2}".format(lidar_num, self.timedata, self.save_counter[lidar_num-1]))
            self.tail_image.save_to_disk("{0}_datacollected_images_{1}/tail/{2}".format(lidar_num, self.timedata, self.save_counter[lidar_num-1]))
            self.right_image.save_to_disk("{0}_datacollected_images_{1}/right/{2}".format(lidar_num, self.timedata, self.save_counter[lidar_num-1]))
            self.lidar_image.save_to_disk("{0}_datacollected_images_{1}/lidar/{2}".format(lidar_num, self.timedata, self.save_counter[lidar_num-1]))
            np.save("{0}/{1}".format(self.save_dir, file_name),np.array(self.lidar_points))
        except Exception as e:
            print(e)
            print("No lidar points to save at counter: {}".format(self.save_counter[lidar_num-1]))

        self.save_counter[lidar_num-1] += 1

    @staticmethod
    def _parse_image(weak_self, image, name):
        self = weak_self()
        if not self:
            return
        if(name != 'lidar'):
            image.convert(self._sensors[self._index][1])
        if name == "head":
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            self.head_image = image
        if name == "tail":
            self.tail_image = image
        if name == "left":
            self.left_image = image
        if name == "right":
            self.right_image = image
        if name == "lidar":
            self.lidar_image = image    
        




# ==============================================================================
# -- ClientSideBoundingBoxes ---------------------------------------------------
# ==============================================================================


class ClientSideBoundingBoxes(object):
    """
    This is a module responsible for creating 3D bounding boxes and drawing them
    client-side on pygame surface.
    """

    @staticmethod
    def get_bounding_boxes(vehicles, camera):
        """
        Creates 3D bounding boxes based on carla vehicle list and camera.
        """
        bounding_boxes = [ClientSideBoundingBoxes.get_bounding_box(vehicle, camera) for vehicle in vehicles]
        threedboxs = [ClientSideBoundingBoxes.get_bounding_box_xyz(vehicle, camera) for vehicle in vehicles]
                
        bounding_boxes = [bb for bb in bounding_boxes if all(bb[:, 2] > 0)]
        
        return [np.asarray(bounding_boxes),np.asarray(threedboxs)]

    @staticmethod
    def draw_bounding_boxes(display, bounding_boxes):
        """
        Draws bounding boxes on pygame display.
        """

        bb_surface = pygame.Surface((VIEW_WIDTH, VIEW_HEIGHT))
        bb_surface.set_colorkey((0, 0, 0))
        for bbox in bounding_boxes:
            points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
            # draw lines
            # base
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
            pygame.draw.line(bb_surface, BB_COLOR, points[1], points[2])
            pygame.draw.line(bb_surface, BB_COLOR, points[2], points[3])
            pygame.draw.line(bb_surface, BB_COLOR, points[3], points[0])
            # top
            pygame.draw.line(bb_surface, BB_COLOR, points[4], points[5])
            pygame.draw.line(bb_surface, BB_COLOR, points[5], points[6])
            pygame.draw.line(bb_surface, BB_COLOR, points[6], points[7])
            pygame.draw.line(bb_surface, BB_COLOR, points[7], points[4])
            # base-top
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[4])
            pygame.draw.line(bb_surface, BB_COLOR, points[1], points[5])
            pygame.draw.line(bb_surface, BB_COLOR, points[2], points[6])
            pygame.draw.line(bb_surface, BB_COLOR, points[3], points[7])
        display.blit(bb_surface, (0, 0))

    @staticmethod
    def get_bounding_box_xyz(vehicle, camera):
        """
        Returns 3D bounding box for a vehicle based on camera view.
        """

        bb_cords = ClientSideBoundingBoxes._create_bb_points(vehicle)
        cords_x_y_z = ClientSideBoundingBoxes._vehicle_to_sensor(bb_cords, vehicle, camera)[:3, :]
        transposed_x_y_z = np.transpose(cords_x_y_z)
        lf = transposed_x_y_z[0,:]  # leftfront
        lb = transposed_x_y_z[1,:] 
        rb = transposed_x_y_z[2,:]
        rf = transposed_x_y_z[3,:]
        cx = (lf[0,0]+lb[0,0]+rb[0,0]+rf[0,0])/4
        cy = (lf[0,1]+lb[0,1]+rb[0,1]+rf[0,1])/4
        slope = cx/cy
        if(np.absolute(slope) > 1):
            transposed_x_y_z[:,1] = -transposed_x_y_z[:,1]
        else:
            transposed_x_y_z[:,0] = -transposed_x_y_z[:,0]
        return transposed_x_y_z

    @staticmethod
    def get_bounding_box(vehicle, camera):
        """
        Returns 3D bounding box for a vehicle based on camera view.
        """

        bb_cords = ClientSideBoundingBoxes._create_bb_points(vehicle)
        cords_x_y_z = ClientSideBoundingBoxes._vehicle_to_sensor(bb_cords, vehicle, camera)[:3, :]

        cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
        bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
        return camera_bbox

    @staticmethod
    def _create_bb_points(vehicle):
        """
        Returns 3D bounding box for a vehicle.
        """

        cords = np.zeros((8, 4))
        extent = vehicle.bounding_box.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords

    @staticmethod
    def _vehicle_to_sensor(cords, vehicle, sensor):
        """
        Transforms coordinates of a vehicle bounding box to sensor.
        """

        world_cord = ClientSideBoundingBoxes._vehicle_to_world(cords, vehicle)
        sensor_cord = ClientSideBoundingBoxes._world_to_sensor(world_cord, sensor)
        # print("World to sensor")
        # print(sensor_cord)
        return sensor_cord

    @staticmethod
    def _vehicle_to_world(cords, vehicle):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """

        bb_transform = carla.Transform(vehicle.bounding_box.location)
        bb_vehicle_matrix = ClientSideBoundingBoxes.get_matrix(bb_transform)
        vehicle_world_matrix = ClientSideBoundingBoxes.get_matrix(vehicle.get_transform())
        bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
        world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _world_to_sensor(cords, sensor):
        """
        Transforms world coordinates to sensor.
        """

        sensor_world_matrix = ClientSideBoundingBoxes.get_matrix(sensor.get_transform())
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

    @staticmethod
    def get_matrix(transform):
        """
        Creates matrix from carla transform.
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix


# ==============================================================================
# -- BasicSynchronousClient ----------------------------------------------------
# ==============================================================================
import time

class BasicSynchronousClient():
    """
    Basic implementation of a synchronous client.
    """

    def __init__(self,delta_seconds,config,lidar_number):
        self.client = None
        self.world = None
        self.camera = None
        self.car = None
        self.delta_seconds = delta_seconds
        self.config = config
        self.lidar_number = lidar_number
        self.display = None
        self.image = None
        self.capture = True
        self.ego = []
        self.save_counter = [0] * self.lidar_number
        self.timedata = time.strftime("%Y-%d-%I-%M-%S",time.localtime(time.time()))
        timedata = self.timedata
        self.dirpath = "datacollected_labels_{}".format(timedata)
        self.cur_dir = os.path.dirname(os.path.realpath(__file__))
        self.save_dir = self.cur_dir +"/"+self.dirpath
#        if not os.path.exists(self.save_dir):
#            os.makedirs(self.save_dir)
        self.bounding_boxes = None

    def camera_blueprint(self):
        """
        Returns camera blueprint.
        """

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        return camera_bp

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """
        
        self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))
        
#        settings = self.world.get_settings()
#        settings.fixed_delta_seconds = 1./30
#        settings.synchronous_mode = synchronous_mode
#        self.world.apply_settings(settings)
        
    def tick(self, clock):
#       self.world.on_tick()
#       self.frame = self.world.tick()
        self.world.on_tick()
        pass

    def setup_car(self,autopilot):
        """
        Spawns actor-vehicle to be controled.
        """

        car_bp = self.world.get_blueprint_library().filter('vehicle.*')[0]
        car_location = random.choice(self.world.get_map().get_spawn_points())
        car_location.location.x = 1.498355
        car_location.location.y = 50.319832
        car_location.location.z = 1.843089
        car_location.rotation.pitch = 0.000000
        car_location.rotation.yaw = -88.891235
        car_location.rotation.roll = 0.000000
        print("Ego vehicle location : {}".format(car_location))

        self.car = self.world.spawn_actor(car_bp, car_location)
        if(autopilot):
            self.car.set_autopilot()
        

    def setup_camera(self):
        """
        Spawns actor-camera to be used to render view.
        Sets calibration for client-side boxes rendering.
        """
        camera_transform = carla.Transform(carla.Location(z=2.8))
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=self.car)
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration
        
        

    def control(self, car):
        """
        Applies control to main car based on pygame pressed keys.
        Will return True If ESCAPE is hit, otherwise False to end main loop.
        """

        keys = pygame.key.get_pressed()        # print(self._sensors[self._index][1])
        image.convert(self._sensors[self._index][1])
        if name == "head":
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            self.head_image = image
        if name == "tail":
            self.tail_image = image
        if name == "left":
            self.left_image = image
        if name == "right":
            self.right_image = image
        if name == "lidar":
            self.lidar_image = image
        if keys[K_ESCAPE]:
            return True

        control = car.get_control()
        control.throttle = 0
        if keys[K_w]:
            control.throttle = 1
            control.reverse = False
        elif keys[K_s]:
            control.throttle = 1
            control.reverse = True
        if keys[K_a]:
            control.steer = max(-1., min(control.steer - 0.05, 0))
        elif keys[K_d]:
            control.steer = min(1., max(control.steer + 0.05, 0))
        else:
            control.steer = 0
        control.hand_brake = keys[K_SPACE]

        car.apply_control(control)
        return False

    @staticmethod
    def set_image(weak_self, img):
        """
        Sets image coming from camera sensor.
        The self.capture flag is a mean of synchronization - once the flag is
        set, next coming image will be stored.
        """

        self = weak_self()
        if self.capture:
            self.image = img
            self.capture = False

    def render(self, display):
        """
        Transforms image from camera sensor and blits it to main pygame display.
        """

        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

        if self._mini_view_image1 is not None:
          
            self.world_instance.camera_manager[0].render(display)
            # self._display.blit(surface, (gap_x, mini_image_y))



    def save_lidar_and_bboxes(self,lidar_num):
        try:
            # changed_boxes = np.multiply(self.bounding_boxes, 0.01)
            file_name = "{0:06d}".format(self.save_counter[lidar_num-1])
            save_dir = self.cur_dir + "/{}_".format(lidar_num) + self.dirpath
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
                
            np.save("{0}/{1}".format(save_dir, file_name),self.threedboxes)
            
            if(lidar_num==1):
                bb_classes = []
                for vehicle in self.vehicles:
                    if('bmw' in str(vehicle.type_id)):
                        bb_classes.append('c')    # car
                    elif('harley' in str(vehicle.type_id)):
                        bb_classes.append('b')    # bike
                    else:
                        bb_classes.append('p')  # pedestrian
                    
                ego_location = self.car.get_location()
                ego_location = [ego_location.x,ego_location.y,ego_location.z]
                
                for i in range(len(bb_classes)-1):
#                    self.lis.append([bb_classes[i],(self.threedboxes[i]-ego_location).tolist()])
#                    x_coord = (self.threedboxes[i][0] - ego_location)[0]
#                    y_coord = (self.threedboxes[i][0] - ego_location)[1]
                    x_coord = self.threedboxes[i][0][0]
                    y_coord = self.threedboxes[i][0][1]
                    if(np.abs(x_coord) < ROI/2 and np.abs(y_coord) < ROI/2):
                        self.lis.append([bb_classes[i],(self.threedboxes[i,:,:]).tolist()])
                
        except:
            print("No BBoxes to save at counter: {}".format(self.save_counter[lidar_num-1]))
        self.save_counter[lidar_num-1] += 1
        self.world_instance.camera_manager[lidar_num].save_lidar_to_file(lidar_num)
    
    
        
        
    def game_loop(self):
        """
        Main program loop.
        """
        self.agent = "Roaming"
        self._mini_view_image1 = True

        try:
            pygame.init()
            pygame.font.init()
            world = None

            # gap_x = (WINDOW_WIDTH - 2 * MINI_WINDOW_WIDTH) / 3
            gap_x = 7
            # mini_image_y = WINDOW_HEIGHT - MINI_WINDOW_HEIGHT - gap_x
            mini_image_y = MINI_WINDOW_HEIGHT - 20

            self.client = carla.Client('127.0.0.1', 2000)
            self.client.set_timeout(5.0)

            self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT*2), pygame.HWSURFACE | pygame.DOUBLEBUF)

            self.world = self.client.get_world()
            self.width = VIEW_WIDTH
            self.height = VIEW_HEIGHT
            
            self.setup_car(True)
            self.world_instance = World(self.world, self.width, self.height, self.car, gap_x, mini_image_y,self.delta_seconds,
                                        self.config,self.lidar_number)
            self.setup_camera()

            pygame_clock = pygame.time.Clock()

            self.set_synchronous_mode(True)
            self.vehicles = self.world.get_actors().filter('vehicle.*')
            self.lis = []

#            controller = KeyboardControl(self.world_instance, True)

            skip_frame_till = 5
            current_skip_num = 0
            
            
            while True:

#                if controller.parse_events(self.client, self.world_instance, pygame_clock):
#                    return

                self.world_instance.tick(pygame_clock)

                self.capture = True
                pygame_clock.tick_busy_loop(20)

                self.render(self.display)
                self.bounding_boxes, self.threedboxes = ClientSideBoundingBoxes.get_bounding_boxes(self.vehicles, self.camera)  
                
                    
                if current_skip_num >= skip_frame_till:
                    for i in range(self.lidar_number):
                        self.save_lidar_and_bboxes(i)  
                    current_skip_num = 0
                

                ClientSideBoundingBoxes.draw_bounding_boxes(self.display, self.bounding_boxes)
                

                pygame.display.flip()

                pygame.event.pump()

                # control = agent.run_step()
                # self.world_instance.player.apply_control(control)
                # if self.control(self.car):
                #     return
                current_skip_num +=1

        finally:
#            self.set_synchronous_mode(False)
            with open('bb_classes_2.txt', 'w') as f:
                json.dump(self.lis,f)
            self.camera.destroy()
            self.car.destroy()
            print("renaming files ...")
            print("Done")
            pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    Initializes the client-side bounding box demo.
    """

    try:
        delta_sec = 1/30
#        with open('lidar_config.txt') as f:
#            lidar_config = json.load(f)
#        index = len(lidar_config['lidar'])
        config = [-1.232, -0.03584, 2.3013, 0.14621, 0.1404, 2.1720, 
                  1.43551, 0.694, 2.25, 3.1415, 3.0349, 2.779]
        index = int(len(config)/6)
        client = BasicSynchronousClient(delta_sec,config,index)
        client.game_loop()
    finally:
        print('EXIT')

if __name__ == '__main__':
    main()
