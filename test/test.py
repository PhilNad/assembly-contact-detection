import sys
import pathlib
sys.path.append(pathlib.Path(__file__).parent.resolve().as_posix() + "/../build/python_bindings/")
import open3d as o3d
import numpy as np
import assembly_cd as acd
import time

obj1_name = "cube1"
obj2_name = "cone"
obj_names = [obj1_name, obj2_name]
object_geometries = []

SHOW_VOXELS = True

if "cube1" in obj_names:
    cube1 = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
    cube1.scale(1, center=cube1.get_center())
    cube1.translate((0, 0, 0))

if "cube2" in obj_names:
    cube2 = o3d.geometry.TriangleMesh.create_box(width=0.5, height=1.0, depth=1.0)
    cube2.scale(1, center=cube2.get_center())
    R = cube2.get_rotation_matrix_from_xyz((0, 3*np.pi/4, 0))
    cube2.rotate(R, center=np.array([0.5, 0.5, 1]))

if "cube3" in obj_names:
    cube3 = o3d.geometry.TriangleMesh.create_box(width=0.5, height=0.5, depth=1.0)
    cube3.translate((0.25, 0.25, 1))
    R = cube3.get_rotation_matrix_from_xyz((0, 0, np.pi/4))
    cube3.rotate(R, center=np.array([0.5, 0.5, 1]))

if "cone" in obj_names:
    cone = o3d.geometry.TriangleMesh.create_cone(radius=0.5, height=1.0)
    cone.translate((0.5, 0.5, 1))


start_time = time.time()
physxScene = acd.Scene()
print("Scene created in: ", time.time() - start_time)

start_time = time.time()

if "cube1" in obj_names:
    #Arguments: id, pose, vertices, triangles, is_fixed, mass, com , material_name
    physxScene.add_object("cube1", 
                            np.identity(4), 
                            np.asarray(cube1.vertices), 
                            np.asarray(cube1.triangles),
                            5, 
                            False, 
                            1.0, 
                            np.array([0, 0, 0]), #CoM is expressed wrt local frame 
                            "wood")
    print("Cube1 added in: ", time.time() - start_time)
    cube1_vertices = physxScene.get_tri_vertices("cube1")
    cube1_triangles = physxScene.get_tri_triangles("cube1")
    cube1 = o3d.geometry.TriangleMesh()
    cube1.vertices  = o3d.utility.Vector3dVector(cube1_vertices)
    cube1.triangles = o3d.utility.Vector3iVector(cube1_triangles)
    object_geometries.append(cube1)

if "cone" in obj_names:
    start_time = time.time()
    physxScene.add_object("cone",
                            np.identity(4),
                            np.asarray(cone.vertices),
                            np.asarray(cone.triangles),
                            30,
                            False,
                            1.0,
                            np.array([0, 0, 0]), #CoM is expressed wrt local frame 
                            "wood")
    print("Cone added in: ", time.time() - start_time)
    cone_vertices = physxScene.get_tri_vertices("cone")
    cone_triangles = physxScene.get_tri_triangles("cone")
    cone = o3d.geometry.TriangleMesh()
    cone.vertices  = o3d.utility.Vector3dVector(cone_vertices)
    cone.triangles = o3d.utility.Vector3iVector(cone_triangles)
    object_geometries.append(cone)

    cone_pose = physxScene.get_object_pose("cone")
    #Translate the cone to the right
    cone_pose[0, 3] += 0.5
    physxScene.set_object_pose("cone", cone_pose)
    cone_pose = physxScene.get_object_pose("cone")
    cone.transform(cone_pose)

if "cube2" in obj_names:    
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
    cube2_vertices = physxScene.get_tri_vertices("cube2")
    cube2_triangles = physxScene.get_tri_triangles("cube2")
    cube2 = o3d.geometry.TriangleMesh()
    cube2.vertices  = o3d.utility.Vector3dVector(cube2_vertices)
    cube2.triangles = o3d.utility.Vector3iVector(cube2_triangles)
    object_geometries.append(cube2)

if "cube3" in obj_names:
    start_time = time.time()
    physxScene.add_object("cube3", 
                            np.identity(4), 
                            np.asarray(cube3.vertices), 
                            np.asarray(cube3.triangles),
                            30, 
                            False, 
                            1.0, 
                            np.array([0, 0, 0]), #CoM is expressed wrt local frame 
                            "wood")
    print("Cube3 added in: ", time.time() - start_time)
    cube3_vertices = physxScene.get_tri_vertices("cube3")
    cube3_triangles = physxScene.get_tri_triangles("cube3")
    cube3 = o3d.geometry.TriangleMesh()
    cube3.vertices  = o3d.utility.Vector3dVector(cube3_vertices)
    cube3.triangles = o3d.utility.Vector3iVector(cube3_triangles)
    object_geometries.append(cube3)

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


contact_points = physxScene.get_contact_points(obj1_name, obj2_name)

contact_point_cloud = o3d.geometry.PointCloud()
contact_point_cloud.points = o3d.utility.Vector3dVector(contact_points)
contact_point_cloud.paint_uniform_color([1, 0, 0])


geometries = object_geometries + [contact_point_cloud]
obj1 = object_geometries[0]
obj2 = object_geometries[1]

if SHOW_VOXELS:
    voxels = []
    #Add voxels from the first object
    obj1_voxel_positions = physxScene.get_voxel_centres(obj1_name)
    obj1_voxel_side_lengths = physxScene.get_voxel_side_lengths(obj1_name)
    obj1_aabb   = obj1.get_axis_aligned_bounding_box()
    obj1_centre = obj1_aabb.get_center()
    obj1_origin = obj1_aabb.get_min_bound()
    obj1_extents = obj1_aabb.get_extent()
    #Create a point cloud of the voxel centres
    obj1_voxel_centres = o3d.geometry.PointCloud()
    obj1_voxel_centres.points = o3d.utility.Vector3dVector(obj1_voxel_positions)
    contact_point_cloud.paint_uniform_color([0, 1, 0])
    obj1_voxelgrid = o3d.geometry.VoxelGrid.create_from_point_cloud(obj1_voxel_centres, voxel_size=obj1_voxel_side_lengths[0])
    voxels.append(obj1_voxel_centres)

    #Add voxels from the second object
    obj2_voxel_positions = physxScene.get_voxel_centres(obj2_name)
    obj2_voxel_side_lengths = physxScene.get_voxel_side_lengths(obj2_name)
    obj2_aabb   = obj2.get_axis_aligned_bounding_box()
    obj2_centre = obj2_aabb.get_center()
    obj2_origin = obj2_aabb.get_min_bound()
    obj2_extents = obj2_aabb.get_extent()
    #Create a point cloud of the voxel centres
    obj2_voxel_centres = o3d.geometry.PointCloud()
    obj2_voxel_centres.points = o3d.utility.Vector3dVector(obj2_voxel_positions)
    contact_point_cloud.paint_uniform_color([0, 0, 1])
    obj2_voxelgrid = o3d.geometry.VoxelGrid.create_from_point_cloud(obj2_voxel_centres, voxel_size=obj2_voxel_side_lengths[0])
    #voxels.append(obj2_voxel_centres)

    #Check if the centres from the first object are inside the voxel grid of the second object
    obj1_included_indices = obj1_voxelgrid.check_if_included(o3d.utility.Vector3dVector(obj2_voxel_positions))
    #Check if the centres from the second object are inside the voxel grid of the first object
    obj2_included_indices = obj2_voxelgrid.check_if_included(o3d.utility.Vector3dVector(obj1_voxel_positions))


    geometries += voxels

o3d.visualization.draw(geometries)