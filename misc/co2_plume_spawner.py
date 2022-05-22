#!/usr/bin/env python3
from stl import mesh
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import random
from skimage import measure
from skimage.draw import ellipsoid

ros1 = True
try:
  import rospy  # ros1
except ModuleNotFoundError:
  ros1 = False
  import rclpy  # ros2
  from gazebo_msgs.srv import SpawnEntity

from geometry_msgs.msg import Point, Pose
from gazebo_msgs.srv import SpawnModel

"""
To try it out try something like this
docker exec -it `docker ps | grep dragonfly-sim | cut -d" " -f1` /bin/bash
source /opt/ros/galactic/setup.bash
cd workspace
python3.8 ./misc/co2_plume_spawner.py
"""

THRESHOLD = 300  # ppm
node = None
spawn_model = None
spawn_entity_client = None


def plume(x, y, z, q=5000, k=2, u=1):
  return (q / (2 * math.pi * k * x)) * np.exp(- (u * (pow(y, 2) + pow(z, 2)) / (4 * k * x)))


def plume_grid():
  x, y, z = np.mgrid[000.1:100:1, -20:20:1, 0:400:1]
  return plume(x, y, z)


plume_base = plume_grid()

xml = open("./gazebo/src/dragonfly_sim/models/co2_shell/model.sdf", "r").read().replace("\n", "")

if ros1:
  spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
  rospy.init_node("co2_plume_spawner")
else:
  rclpy.init()
  node = rclpy.create_node('co2_plume_spawner')
  spawn_entity_client = node.create_client(srv_type=SpawnEntity, srv_name='spawn_entity')
  spawn_entity_client.wait_for_service(timeout_sec=1.0)

for threshold_div in range(1, 6):
  # Use marching cubes to obtain the surface mesh of these ellipsoids
  vertices, faces, _, values = measure.marching_cubes(plume_base, THRESHOLD / threshold_div)
  cube = mesh.Mesh(np.zeros(2 * faces.shape[0], dtype=mesh.Mesh.dtype))
  for i, f in enumerate(faces):
    for j in range(3):
      cube.vectors[i][j] = vertices[f[j], :]
  for i, f in enumerate(faces):
    for j in range(3):
      cube.vectors[i + len(faces)][j] = vertices[f[2-j], :]

  stl_filename = '/tmp/plume' + str(threshold_div) + '.stl'
  cube.save(stl_filename)
  plume_co2_avg = sum(values) / len(values)

  new_xml = xml.replace("<uri>model://co2_shell/cube.stl</uri>", "<uri>{}</uri>".format(stl_filename)).replace(
    "<ambient>0 0 0 1</ambient>", "<ambient>{red} {green} {blue} {trans}</ambient>".format(
      red=(6 - threshold_div) / 6.0,
      green=0,
      blue=threshold_div / 6.0,
      trans=0.5))
  if ros1:
    spawn_model("plume" + str(threshold_div), new_xml, "", Pose(), "world")
  else:  # ros2
    req = SpawnEntity.Request(xml=new_xml)
    req.name = "plume" + str(threshold_div)
    req.initial_pose.position = Point(x=0.0, y=0.0, z=0.0)  # shell_offset
    spawn_entity_client.call_async(req)
if not ros1:
  rclpy.spin_once(node)
print("Done")
