import sys
import pathlib
sys.path.append(pathlib.Path(__file__).parent.resolve().as_posix() + "/../build/python_bindings/")
import open3d as o3d
import numpy as np
import assembly_cd as acd
import time

""" vert = np.array([
    [-1, -1, -1],
    [-1,  1, -1],
    [ 1,  1, -1],
    [ 1, -1, -1],
    [-1, -1,  1],
    [-1,  1,  1],
    [ 1,  1,  1],
    [ 1, -1,  1]
])

tri = np.array([
    [4, 6, 7],
    [4, 6, 5],
    [0, 2, 3],
    [0, 2, 1],
    [7, 2, 3],
    [7, 2, 6],
    [4, 1, 0],
    [4, 1, 5],
    [5, 2, 1],
    [5, 2, 6],
    [4, 3, 0],
    [4, 3, 7]
])

tm = o3d.geometry.TriangleMesh()
tm.vertices  = o3d.utility.Vector3dVector(vert)
tm.triangles = o3d.utility.Vector3iVector(tri)
tm.orient_triangles()
print(np.asarray(tm.triangles))
print('Volume: ', tm.get_volume())
print('Water tight: ', tm.is_watertight())
o3d.visualization.draw([tm]) """


cube1 = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
cube1.scale(1, center=cube1.get_center())
cube1.translate((0, 0, 0))

cube2 = o3d.geometry.TriangleMesh.create_box(width=0.5, height=1.0, depth=1.0)
cube2.scale(1, center=cube2.get_center())
R = cube2.get_rotation_matrix_from_xyz((0, 3*np.pi/4, 0))
cube2.rotate(R, center=np.array([0.5, 0.5, 1]))

cube3 = o3d.geometry.TriangleMesh.create_box(width=0.5, height=0.5, depth=1.0)
cube3.translate((0.25, 0.25, 1))
R = cube3.get_rotation_matrix_from_xyz((0, 0, np.pi/4))
cube3.rotate(R, center=np.array([0.5, 0.5, 1]))

cone = o3d.geometry.TriangleMesh.create_cone(radius=0.5, height=1.0)
cone.translate((0.5, 0.5, 1))

#o3d.visualization.draw([cube1, cube2])
#exit()

start_time = time.time()
physxScene = acd.Scene()
print("Scene created in: ", time.time() - start_time)

start_time = time.time()
#Arguments: id, pose, vertices, triangles, is_fixed, mass, com , material_name
physxScene.add_object("cube1", 
                        np.identity(4), 
                        np.asarray(cube1.vertices), 
                        np.asarray(cube1.triangles),
                        30, 
                        False, 
                        1.0, 
                        np.array([0, 0, 0]), #CoM is expressed wrt local frame 
                        "wood")
print("Cube1 added in: ", time.time() - start_time)

# start_time = time.time()
# physxScene.add_object("cone",
#                         np.identity(4),
#                         np.asarray(cone.vertices),
#                         np.asarray(cone.triangles),
#                         30,
#                         False,
#                         1.0,
#                         np.array([0, 0, 0]), #CoM is expressed wrt local frame 
#                         "wood")
# print("Cone added in: ", time.time() - start_time)

start_time = time.time()
physxScene.add_object("cube2", 
                        np.identity(4), 
                        np.asarray(cube2.vertices), 
                        np.asarray(cube2.triangles),
                        30, 
                        False, 
                        1.0, 
                        np.array([0, 0, 0]), #CoM is expressed wrt local frame 
                        "wood")
print("Cube2 added in: ", time.time() - start_time)

# start_time = time.time()
# physxScene.add_object("cube3", 
#                         np.identity(4), 
#                         np.asarray(cube3.vertices), 
#                         np.asarray(cube3.triangles),
#                         30, 
#                         False, 
#                         1.0, 
#                         np.array([0, 0, 0]), #CoM is expressed wrt local frame 
#                         "wood")
# print("Cube3 added in: ", time.time() - start_time)

'''
vert = physxScene.get_tri_vertices("cube1")
tri = physxScene.get_tri_triangles("cube1")
tm = o3d.geometry.TriangleMesh()
tm.vertices  = o3d.utility.Vector3dVector(vert)
tm.triangles = o3d.utility.Vector3iVector(tri)
'''

""" vert = physxScene.get_tetra_vertices("cube1")
tetra = physxScene.get_tetra_indices("cube1").astype(np.int64)
tm = o3d.geometry.TetraMesh(o3d.utility.Vector3dVector(vert), o3d.utility.Vector4iVector(tetra))
o3d.visualization.draw_geometries([tm]) """

'''
voxels = []
#Add voxels from cube1
voxel_positions = physxScene.get_voxel_centres("cube1")
voxel_side_lengths = physxScene.get_voxel_side_lengths("cube1")
for i in range(len(voxel_positions)):
    cube = o3d.geometry.TriangleMesh.create_box(width=voxel_side_lengths[0], height=voxel_side_lengths[1], depth=voxel_side_lengths[2])
    cube.scale(1, center=cube.get_center())
    cube.translate(voxel_positions[i], relative=False)
    voxels.append(cube)

#Add voxels from cone
voxel_positions = physxScene.get_voxel_centres("cone")
voxel_side_lengths = physxScene.get_voxel_side_lengths("cone")
for i in range(len(voxel_positions)):
    cube = o3d.geometry.TriangleMesh.create_box(width=voxel_side_lengths[0], height=voxel_side_lengths[1], depth=voxel_side_lengths[2])
    cube.scale(1, center=cube.get_center())
    cube.translate(voxel_positions[i], relative=False)
    voxels.append(cube)

o3d.visualization.draw([tm]+voxels)
'''

#print(physxScene.get_contacted_objects("cube1"))
#print(physxScene.get_contacted_objects("cone"))
#physxScene.merge_similar_contact_points()
contact_points = physxScene.get_contact_points("cube1", "cube2")

contact_point_cloud = o3d.geometry.PointCloud()
contact_point_cloud.points = o3d.utility.Vector3dVector(contact_points)
o3d.visualization.draw([cube1, cube2, contact_point_cloud])