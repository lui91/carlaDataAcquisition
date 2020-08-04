import glob
import os
import sys
from pathlib import Path

try:
    os = 'win-amd64' if os.name == 'nt' else 'linux-x86_64'
    path = '/opt/carla-simulator/PythonAPI/carla/dist/'
    carla_egg = f'carla-0.9.9-py{sys.version_info.major}.{sys.version_info.minor}-{os}.egg'
    carla_path = glob.glob(f'{path}{carla_egg}')[0]
    sys.path.append(carla_path)
except IndexError:
    pass

import carla
import argparse
import logging
import random


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
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:

        world = client.get_world()
        ego_vehicle = None
        ego_cam = None
        ego_col = None
        ego_Depth = None
        ego_obs = None
        lidar_sen = None
        sem_cam = None
        # --------------
        # Start recording
        # --------------
        """
        client.start_recorder('~/tutorial/recorder/recording01.log')
        """

        # --------------
        # Spawn ego vehicle
        # --------------
        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name', 'ego')
        print('\nEgo role_name is set')
        ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        ego_bp.set_attribute('color', ego_color)
        print('\nEgo color is set')

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            ego_transform = spawn_points[0]
            ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
            print('\nEgo is spawned')
        else:
            logging.warning('Could not found any spawn points')

        # --------------
        # Add a RGB camera sensor to ego vehicle.
        # --------------
        cam_bp = None
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", str(1920))
        cam_bp.set_attribute("image_size_y", str(1080))
        cam_bp.set_attribute("fov", str(105))
        cam_location = carla.Location(2,0,1)
        cam_rotation = carla.Rotation(0,0,0)
        cam_transform = carla.Transform(cam_location,cam_rotation)
        ego_cam = world.spawn_actor(cam_bp, cam_transform, attach_to=ego_vehicle)
        ego_cam.listen(lambda image: image.save_to_disk('./output/RGB/%.6d.jpg' % image.frame))

        # --------------
        # Add a Depth camera sensor to ego vehicle.
        # --------------
        cam_depth = None
        cam_depth = world.get_blueprint_library().find('sensor.camera.depth')
        cam_depth.set_attribute("image_size_x", str(1920))
        cam_depth.set_attribute("image_size_y", str(1080))
        cam_depth.set_attribute("fov", str(105))
        cam_location = carla.Location(2, 0, 1)
        cam_rotation = carla.Rotation(0, 0, 0)
        cam_transform = carla.Transform(cam_location, cam_rotation)
        ego_Depth = world.spawn_actor(cam_depth, cam_transform, attach_to=ego_vehicle)
        # normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
        # in_meters = 1000 * normalized
        # ccDepth = carla.ColorConverter.Depth
        ccLogDepth = carla.ColorConverter.LogarithmicDepth
        # ego_Depth.listen(lambda image: image.save_to_disk('./output/Depth/%.6d.jpg' % image.frame))
        ego_Depth.listen(lambda image: image.save_to_disk('./output/LogDepth/%.6d.jpg' % image.frame, ccLogDepth))

        # --------------
        # Add a new LIDAR sensor to my ego
        # --------------
        lidar_cam = None
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels',str(40))
        # lidar_bp.set_attribute('upper_fov',str(30))
        # lidar_bp.set_attribute('upper_fov', str(-50))
        lidar_bp.set_attribute('points_per_second',str(120000))
        lidar_bp.set_attribute('rotation_frequency',str(40))
        lidar_bp.set_attribute('range',str(40))
        lidar_location = carla.Location(0,0,2)
        lidar_rotation = carla.Rotation(0,0,0)
        lidar_transform = carla.Transform(lidar_location,lidar_rotation)
        lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle)
        lidar_sen.listen(lambda point_cloud: point_cloud.save_to_disk('./output/LIDAR/%.6d.ply' % point_cloud.frame))

        # --------------
        # Add a new semantic segmentation camera to my ego
        # --------------
        sem_cam = None
        sem_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        sem_bp.set_attribute("image_size_x",str(1920))
        sem_bp.set_attribute("image_size_y",str(1080))
        sem_bp.set_attribute("fov",str(105))
        sem_location = carla.Location(2,0,1)
        sem_rotation = carla.Rotation(0,0,0)
        sem_transform = carla.Transform(sem_location,sem_rotation)
        sem_cam = world.spawn_actor(sem_bp,sem_transform,attach_to=ego_vehicle)
        # This time, a color converter is applied to the image, to get the semantic segmentation view
        sem_cam.listen(lambda image: image.save_to_disk('./output/2dSemantic/%.6d.jpg' % image.frame,carla.ColorConverter.CityScapesPalette))


        # --------------
        # Place spectator on ego spawning
        # --------------

        spectator = world.get_spectator()
        world_snapshot = world.wait_for_tick() 
        spectator.set_transform(ego_vehicle.get_transform())

        # --------------
        # Enable autopilot for ego vehicle
        # --------------

        ego_vehicle.set_autopilot(True)

        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        while True:
            world_snapshot = world.wait_for_tick()

    finally:
        # --------------
        # Stop recording and destroy actors
        # --------------
        client.stop_recorder()
        if ego_vehicle is not None:
            if ego_cam is not None:
                ego_cam.stop()
                ego_cam.destroy()
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if ego_Depth is not None:
                ego_Depth.stop()
                ego_Depth.destroy()
            if ego_obs is not None:
                ego_obs.stop()
                ego_obs.destroy()
            if lidar_sen is not None:
                lidar_sen.stop()
                lidar_sen.destroy()
            if sem_cam is not None:
                sem_cam.stop()
                sem_cam.destroy()
            ego_vehicle.destroy()


def delete_folder(del_path):
    for sub in del_path.iterdir():
        if sub.is_dir() :
                delete_folder(sub)
        else :
            sub.unlink()

if __name__ == '__main__':
    delete = True
    try:
        if delete:
            test = Path('/home/luis/PycharmProjects/pythonProject/output/')
            delete_folder(test)
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with tutorial_ego.')