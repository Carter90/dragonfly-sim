# @SEE https://pythonhosted.org/numpy-stl/usage.html

import numpy
from stl import mesh
from geometry_msgs.msg import Point


class Shell:
    def __init__(self,
                 co2_value=0.0,
                 shell_offset=Point(x=0.0, y=0.0, z=0.0),
                 faces_list=numpy.array(
                     [[0, 3, 1], [1, 3, 2], [0, 4, 7], [0, 7, 3], [4, 5, 6], [4, 6, 7], [5, 1, 2], [5, 2, 6], [2, 3, 6],
                      [3, 7, 6], [0, 1, 5], [0, 5, 4]]),
                 vertices_list=numpy.array(
                     [[-1, -1, -1], [+1, -1, -1], [+1, +1, -1], [-1, +1, -1], [-1, -1, +1], [+1, -1, +1], [+1, +1, +1],
                      [-1, +1, +1]])
                 ):
        self.co2_value = co2_value
        self.shell_offset = shell_offset
        self.faces_list = faces_list
        self.vertices_list = vertices_list


# -----------This would get generated elsewhere

# @NOTES
# Define the 8 vertices of the cube
vertices = numpy.array(
    [[-1, -1, -1], [+1, -1, -1], [+1, +1, -1], [-1, +1, -1], [-1, -1, +1], [+1, -1, +1], [+1, +1, +1], [-1, +1, +1]])
# Define the 12 triangles composing the cube
faces = numpy.array(
    [[0, 3, 1], [1, 3, 2], [0, 4, 7], [0, 7, 3], [4, 5, 6], [4, 6, 7], [5, 1, 2], [5, 2, 6], [2, 3, 6], [3, 7, 6],
     [0, 1, 5], [0, 5, 4]])

# just randomly scale them down
plume = [Shell(), Shell(), Shell()]
for plume_shell in plume:
    plume_shell.faces_list = plume_shell.faces_list / (plume.index(plume_shell) + 1)
    plume_shell.faces_list = plume_shell.faces_list / (plume.index(plume_shell) + 1)

# ----------- End This would get generated elsewhere

for plume_shell in plume:
    # Create the mesh
    cube = mesh.Mesh(numpy.zeros(plume_shell.faces_list.shape[0], dtype=mesh.Mesh.dtype))
    for i, f in enumerate(plume_shell.faces_list):
        for j in range(3):  # plume_shell.faces_list.shape[0]+1
            cube.vectors[i][j] = plume_shell.vertices_list[f[j], :]

    # Write the mesh to file
    cube.save('/tmp/shell{}.stl'.format(plume.index(plume_shell)))

import rclpy
from geometry_msgs.msg import Point
from gazebo_msgs.srv import SpawnEntity

xml = open("/home/carter/model_editor_models/co2_shell/model.sdf", "r").read().replace("\n", "")
rclpy.init()
node = rclpy.create_node('co2_plume_spawner')
spawn_entity_client = node.create_client(srv_type=SpawnEntity, srv_name='spawn_entity')
spawn_entity_client.wait_for_service(timeout_sec=1.0)
req = SpawnEntity.Request()
req.xml = xml
req.initial_pose.position = Point(x=0.0, y=0.0, z=0.0)  # shell_offset
spawn_entity_client.call_async(req)
