import sys
import pathlib
sys.path.insert(0, pathlib.Path(__file__).parent.resolve().as_posix() + "/../build/python_bindings/")
import open3d as o3d
import numpy as np
import assembly_cd as acd


#Instanciate the objects
cube = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
cone = o3d.geometry.TriangleMesh.create_cone(radius=0.5, height=1.0)

#Set the poses of the objects such that the cone is atop the cube
cube_pose = np.identity(4)
cone_pose = np.identity(4)
cone_pose[0:3, 3] = [0.5, 0.5, 1]

#Instanciate the scene and add objects
scene = acd.Scene()
scene.add_object("cube", cube_pose, np.asarray(cube.vertices), np.asarray(cube.triangles), 30, is_fixed=True)
scene.add_object("cone", cone_pose, np.asarray(cone.vertices), np.asarray(cone.triangles), 30, mass=1, com=[0, 0, 0.5])

#Detect contact points between the two objects
contact_points = scene.get_contact_point_positions("cube", "cone")

#Get the contact forces
contact_forces = scene.get_contact_forces()

#Draw the scene and the contact points
# Tip: In the GUI menu, select Actions and click on 'Show Settings'. 
# Then, in the settings panel, select 'Wireframe' to see through objects,
# and set the background color to white.
cone.transform(cone_pose)
contact_point_cloud = o3d.geometry.PointCloud()
contact_point_cloud.points = o3d.utility.Vector3dVector(contact_points)
o3d.visualization.draw([cube, cone, contact_point_cloud])