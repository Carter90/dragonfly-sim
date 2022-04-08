from stl import mesh
from geometry_msgs.msg import Point
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import random
from skimage import measure
from skimage.draw import ellipsoid
import rclpy
from geometry_msgs.msg import Point
from gazebo_msgs.srv import SpawnEntity

THRESHOLD = 30  # ppm


def plume(x, y, z, q=5000, k=2, u=1):
    return (q / (2 * math.pi * k * x)) * np.exp(- (u * (pow(y, 2) + pow(z, 2)) / (4 * k * x)))


def plume_grid():
    x, y, z = np.mgrid[000.1:100:1, -20:20:1, 0:400:1]
    return plume(x, y, z)


plume_base = plume_grid()

xml = open("/home/carter/dragonfly-sim/gazebo/src/dragonfly_sim/models/co2_shell/model.sdf", "r").read().replace("\n",
                                                                                                                 "")
rclpy.init()
node = rclpy.create_node('co2_plume_spawner2')
spawn_entity_client = node.create_client(srv_type=SpawnEntity, srv_name='spawn_entity')
spawn_entity_client.wait_for_service(timeout_sec=1.0)

rclpy.spin_once(node)
for thres_div in range(1, 6):
    # Use marching cubes to obtain the surface mesh of these ellipsoids
    vertices, faces, _, values = measure.marching_cubes(plume_base, THRESHOLD / thres_div)
    cube = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            cube.vectors[i][j] = vertices[f[j], :]
    stl_filename = '/tmp/plume' + str(thres_div) + '.stl'
    cube.save(stl_filename)
    plume_co2_avg = sum(values) / len(values)
    req = SpawnEntity.Request()
    req.xml = xml.replace("<uri>model://co2_shell/cube.stl</uri>", "<uri>model://{}</uri>".format(stl_filename))
    req.xml = req.xml.replace("<diffuse>0 1 0 0</diffuse>", "<diffuse>{red} {green} {blue} 0</diffuse>".format(
        red=255 / plume_co2_avg,
        green=plume_co2_avg, blue=0))
    req.initial_pose.position = Point(x=0.0, y=0.0, z=0.0)  # shell_offset
    spawn_entity_client.call_async(req)
print("Done")
rclpy.spin(node)

