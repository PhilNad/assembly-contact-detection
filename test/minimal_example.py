import sys
import pathlib
sys.path.append(pathlib.Path(__file__).parent.resolve().as_posix() + "/../build/python_bindings/")
import open3d as o3d
import numpy as np
import assembly_cd as acd


#A cone is atop a cube
cube = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
cone = o3d.geometry.TriangleMesh.create_cone(radius=0.5, height=1.0)
cone.translate((0.5, 0.5, 1))

#Instanciate the scene and add objects
scene = acd.Scene()
scene.add_object("cube", np.identity(4), np.asarray(cube.vertices), np.asarray(cube.triangles), 30)
scene.add_object("cone", np.identity(4), np.asarray(cone.vertices), np.asarray(cone.triangles), 30)

#Detect contact points between the two objects
contact_points = scene.get_contact_points("cube", "cone")

#Draw the scene and the contact points
# Tip: In the GUI menu, select Actions and click on 'Show Settings'. 
# Then, in the settings panel, select 'Wireframe' to see through objects,
# and set the background color to white.
contact_point_cloud = o3d.geometry.PointCloud()
contact_point_cloud.points = o3d.utility.Vector3dVector(contact_points)
o3d.visualization.draw([cube, cone, contact_point_cloud])