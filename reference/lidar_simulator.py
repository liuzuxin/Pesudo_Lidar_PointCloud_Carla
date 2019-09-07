
"""Basic CARLA client to generate point cloud into ros/sensormsg/PointCloud format that you can visualize with RViz.

   You can add a lidar by :

       RSLidar32B_name = 'RSLidar32B'
       RSLidar32B = Lidar(RSLidar32B_name)
       RSLidar32B.set(Channels=32,              # type int,   the beans' number of lidar,usually setting as 32,16
                      Range=200,                # type float, the detection range of lidar,usually setting as 100.,200.
                      PointsPerSecond=640000,   # type int,   the number of points generated in one second,
                                                              # calculated by Channels *
                      RotationFrequency=10,     # type int,   the rotation frequency of lidar,usually setting 10
                      UpperFovLimit=15.0,       # type float, the value of lidar character,  represent the upperfovlimit
                      LowerFovLimit=-15.0,
                      GaussianNoise=3.0,        # type float, the sigma of Gaussian distribution, setting sigma= 1.0
                                                              #means the 99% noise value will be limited in (-3cm,+3cm)
                      DropOutPattern=0.0,       # type float, the level of dropout method, Bigger DropOutPattern is,
                                                              # less points remain. When setting it to 0.0,no points
                                                              # will be dropped.
                      LidarType=0,              # type int,   the model of lidar,
                                                              # 0: normal lidar,vertical angle is separated evenly
                                                              # 1: RS32-B,with unevenly distribution of vertical angle
                                                              # 2: RS MEMES lidar
                      HorizonRange=360.0,       # type float, the detection range of cycle, usually setting 360.
                                                              # and 63.0 when the lidar is MEMS
                      DebugFlag=0               # type int,   Using for debug and you can define its function by modify
                                                              #the lidar.cpp in
                                                              " CarlaUE4/Plugins/Carla/Source/Carla/Sensor/Lidar.cpp"
                      )
  """

from __future__ import print_function

import argparse
import logging
import random
import time
from carla.client import make_carla_client
from carla.sensor import Camera, Lidar
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line, StopWatch


def run_carla_client(pub, host, port):
    print('Attempt to have CarlaClient connected...')
    # Connect with the server
    with make_carla_client(host, port) as client:
        print('CarlaClient connected')

        settings = CarlaSettings()
        settings.set(SynchronousMode=True, SendNonPlayerAgentsInfo=False, NumberOfVehicles=20, NumberOfPedestrians=40,
                     WeatherId=random.choice([1, 3, 7, 8, 14]))
        settings.randomize_seeds()

        lidar1_name = 'LidarMain32B'
        lidar1 = Lidar(lidar1_name)
        lidar1.set(Channels=32, Range=200, PointsPerSecond=640000, RotationFrequency=10, UpperFovLimit=15.0,
                   LowerFovLimit=-25.0, GaussianNoise=3.0, DropOutPattern=0.0, LidarType=1,
                   HorizonRange=360.0, DebugFlag=0)
        lidar1.set_position(x=0, y=0, z=2.50)
        lidar1.set_rotation(pitch=0, yaw=0, roll=0)
        settings.add_sensor(lidar1)

        lidar2_name = 'LidarSub16L'
        lidar2 = Lidar(lidar2_name)
        lidar2.set(Channels=16, Range=150, PointsPerSecond=80000, RotationFrequency=10,UpperFovLimit=15.0, LowerFovLimit=-15.0,
                   GaussianNoise=0.02,
                   DropOutPattern=1.0,
                   LidarType=2
                   )
        lidar2.set_position(x=0, y=0.4, z=2.20)
        lidar2.set_rotation(pitch=0, yaw=0, roll=45)
        settings.add_sensor(lidar2)

        lidar3_name = 'LidarSub16R'
        lidar3 = Lidar(lidar3_name)
        lidar3.set(Channels=16, Range=150, PointsPerSecond=80000, RotationFrequency=10,UpperFovLimit=15.0, LowerFovLimit=-15.0,
                   GaussianNoise=0.02,
                   DropOutPattern=1.0,
                   LidarType=2
                   )
        lidar3.set_position(x=0, y=-0.4, z=2.20)
        lidar3.set_rotation(pitch=0, yaw=0, roll=-45)
        settings.add_sensor(lidar3)

        lidar4_name = 'LidarMain32B_Ref'
        lidar4 = Lidar(lidar4_name)
        lidar4.set(Channels=32, Range=200, PointsPerSecond=640000, RotationFrequency=10, UpperFovLimit=15.0,
                   LowerFovLimit=-25.0, GaussianNoise=0.0, DropOutPattern=0.0, LidarType=1,
                   HorizonRange=360.0, DebugFlag=0)
        lidar4.set_position(x=0, y=0, z=2.50)
        lidar4.set_rotation(pitch=0, yaw=0, roll=0)
        settings.add_sensor(lidar4)

        lidar5_name = 'LidarMEMS'
        lidar5 = Lidar(lidar5_name)
        lidar5.set(Channels=100, Range=200, PointsPerSecond=690000, RotationFrequency=10, UpperFovLimit=10.0, LowerFovLimit=-10.0,
                   GaussianNoise=3.0,
                   DropOutPattern=1.0,
                   LidarType=2,
                   HorizonRange=63.0,
                   DebugFlag=0
                   )
        lidar5.set_position(x=0, y=0, z=2.0)
        lidar5.set_rotation(pitch=0, yaw=0, roll=0)
        settings.add_sensor(lidar5)

        client.load_settings(settings)
        client.start_episode(0)  # Start at location index id '0'

        # Iterate every frame in the episode except for the first one.
        frame = 0
        while True:
            frame += 1
            measurements, sensor_data = client.read_data()

            RSlidar32M_points = sensor_data[lidar1_name].point_cloud.array
            RSlidar32M_points_msg = PointCloud_Gen(RSlidar32M_points)
            pub['RSlidar32M'].publish(RSlidar32M_points_msg)

            RSlidar32Ms_points = sensor_data[lidar4_name].point_cloud.array
            RSlidar32Ms_points_msg = PointCloud_Gen(RSlidar32Ms_points)
            pub['RSlidar32M_Ref'].publish(RSlidar32Ms_points_msg)

            RSlidarMEMS_points = sensor_data[lidar5_name].point_cloud.array
            RSlidarMEMS_points_msg = PointCloud_Gen(RSlidarMEMS_points)
            pub['RSlidarMEMS'].publish(RSlidarMEMS_points_msg)

            RSlidar16L_points = sensor_data[lidar2_name].point_cloud.array
            RSlidar16L_points_msg = PointCloud_Gen(RSlidar16L_points)
            pub['RSlidar16L'].publish(RSlidar16L_points_msg)

            RSlidar16R_points = sensor_data[lidar3_name].point_cloud.array
            RSlidar16R_points_msg = PointCloud_Gen(RSlidar16R_points)
            pub['RSlidar16R'].publish(RSlidar16R_points_msg)

            rospy.loginfo("Send {} frame pointcloud ".format(frame))
            client.send_control(measurements.player_measurements.autopilot_control)


def print_message(elapsed_time, point_n, frame):
    message = ' '.join([
        'Transformations took {:>3.0f} ms.',
        'Saved {:>6} points to "{:0>5}.ply".'
    ]).format(elapsed_time, point_n, frame)
    print_over_same_line(message)


def check_far(value):
    fvalue = float(value)
    if fvalue < 0.0 or fvalue > 1.0:
        raise argparse.ArgumentTypeError(
            "{} must be a float between 0.0 and 1.0")
    return fvalue


def PointCloud_Gen(points, frameID='rslidar'):
    from sensor_msgs.msg import PointCloud, ChannelFloat32
    from geometry_msgs.msg import Point32
    import numpy as np
    ##=========PointCloud===============
    points = np.array(points, np.float32)
    point_cloud = points.reshape((-1, 3))
    pointx = point_cloud[:, 0].flatten()
    pointy = point_cloud[:, 1].flatten()
    pointz = point_cloud[:, 2].flatten() * -1.0
    # intensity = point_cloud[:, 3].flatten()
    intensity = np.ones(pointx.shape, dtype=np.float32)
    # labels = point_cloud[:,6].flatten()

    seg_point = PointCloud()
    seg_point.header.frame_id = frameID
    channels1 = ChannelFloat32()
    seg_point.channels.append(channels1)
    seg_point.channels[0].name = "rgb"
    channels2 = ChannelFloat32()
    seg_point.channels.append(channels2)
    seg_point.channels[1].name = "intensity"

    for i in range(point_cloud.shape[0]):
        seg_point.channels[1].values.append(intensity[i])
        if True:  # labels[i] == 1:
            seg_point.channels[0].values.append(255)
            geo_point = Point32(pointx[i], pointy[i], pointz[i])
            seg_point.points.append(geo_point)
        # else:
        #     seg_point.channels[0].values.append(255255255)
        #     geo_point = Point32(pointx[i], pointy[i], pointz[i])
        #     seg_point.points.append(geo_point)
        # elif result[i] == 2:
        #     seg_point.channels[0].values.append(255255255)
        #     geo_point = Point32(pointx[i], pointy[i], pointz[i])
        #     seg_point.points.append(geo_point)
        # elif result[i] == 3:
        #     seg_point.channels[0].values.append(255000)
        #     geo_point = Point32(pointx[i], pointy[i], pointz[i])
        #     seg_point.points.append(geo_point)

    return seg_point


def main(pub):
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    while True:
        try:
            run_carla_client(pub, host=args.host, port=args.port)
            print('\nDone!')
            return

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)


if __name__ == '__main__':

    import rospy
    from sensor_msgs.msg import PointCloud

    rospy.init_node("Simulator_lidar")
    points_pub1 = rospy.Publisher('RSlidar32M', PointCloud, queue_size=100)
    points_pub2 = rospy.Publisher('RSlidar16L', PointCloud, queue_size=100)
    points_pub3 = rospy.Publisher('RSlidar16R', PointCloud, queue_size=100)
    points_pub4 = rospy.Publisher('RSlidar32M_Ref', PointCloud, queue_size=100)
    points_pub5 = rospy.Publisher('RSlidarMEMS', PointCloud, queue_size=100)
    rospy.loginfo("Ros init done!")

    pub_dict = dict({'RSlidar32M': points_pub1,
                     'RSlidar16L': points_pub2,
                     'RSlidar16R': points_pub3,
                     'RSlidar32M_Ref': points_pub4,
                     'RSlidarMEMS': points_pub5,
                     })
    try:
        main(pub_dict)
    except KeyboardInterrupt:
        print('\nClient stoped by user.')
