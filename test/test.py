import sys
import pathlib
sys.path.append(pathlib.Path(__file__).parent.resolve().as_posix() + "/../build/python_bindings/")
import open3d as o3d
import numpy as np
import assembly_cd as acd
import time

obj1_name = "cube1"
obj2_name = "cube4"
obj_names = [obj1_name, obj2_name]
object_geometries = []

SHOW_VOXELS = False

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

if "cube4" in obj_names:
    cube4 = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)

if "cone" in obj_names:
    cone = o3d.geometry.TriangleMesh.create_cone(radius=0.5, height=1.0)


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
                            30,
                            True, 
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
    pose = np.identity(4)
    pose[0:3, 3] = np.array([0.5, 0.5, 1])
    physxScene.add_object("cone",
                            pose,
                            np.asarray(cone.vertices),
                            np.asarray(cone.triangles),
                            30,
                            True,
                            False,
                            1.0,
                            np.array([0, 0, 0.5]), #CoM is expressed wrt local frame 
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
    cone_pose[0, 3] += 0.25
    #cone_pose[2, 3] -= 0.1
    start_time = time.time()
    physxScene.set_object_pose("cone", cone_pose)
    cone_pose = physxScene.get_object_pose("cone")
    print("Cone moved in: ", time.time() - start_time)
    cone.transform(cone_pose)

if "cube2" in obj_names:    
    start_time = time.time()
    physxScene.add_object("cube2", 
                            np.identity(4), 
                            np.asarray(cube2.vertices), 
                            np.asarray(cube2.triangles),
                            30, 
                            True,
                            False, 
                            1.0, 
                            np.array([0, 0, 0]), #CoM must be expressed wrt local frame, NOT THE CASE HERE
                            "wood")
    physxScene.add_object("cube2", 
                            np.identity(4), 
                            np.asarray(cube2.vertices), 
                            np.asarray(cube2.triangles),
                            30, 
                            True,
                            False, 
                            1.0, 
                            np.array([0, 0, 0]), #CoM must be expressed wrt local frame, NOT THE CASE HERE
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
                            True, 
                            False, 
                            1.0, 
                            np.array([0, 0, 0]), #CoM must be expressed wrt local frame, NOT THE CASE HERE
                            "wood")
    print("Cube3 added in: ", time.time() - start_time)
    cube3_vertices = physxScene.get_tri_vertices("cube3")
    cube3_triangles = physxScene.get_tri_triangles("cube3")
    cube3 = o3d.geometry.TriangleMesh()
    cube3.vertices  = o3d.utility.Vector3dVector(cube3_vertices)
    cube3.triangles = o3d.utility.Vector3iVector(cube3_triangles)
    object_geometries.append(cube3)

if "cube4" in obj_names:
    pose = np.identity(4)
    pose[0:3, 3] = np.array([0, 0, 0.1])
    physxScene.add_object("cube4", 
                        pose, 
                        np.asarray(cube4.vertices), 
                        np.asarray(cube4.triangles),
                        30,
                        True, 
                        False, 
                        1.0, 
                        np.array([0.5, 0.5, 0.5]), #CoM is expressed wrt local frame 
                        "wood")
    
    pose = np.identity(4)
    pose[0:3, 3] = np.array([0, 0, 0.5])
    physxScene.set_object_pose("cube4", pose)
    pose = np.identity(4)
    pose[0:3, 3] = np.array([0, 0, 0.9])
    physxScene.set_object_pose("cube4", pose)

    cube4_pose = physxScene.get_object_pose("cube4")
    cube4.transform(cube4_pose)
    object_geometries.append(cube4)
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

geometries = []

contact_points = physxScene.get_contact_point_positions(obj1_name, obj2_name)
print("Number of contact points: ", len(contact_points))

if len(contact_points) > 0:
    contact_point_cloud = o3d.geometry.PointCloud()
    contact_point_cloud.points = o3d.utility.Vector3dVector(contact_points)
    contact_point_cloud.paint_uniform_color([0, 0, 1])
    geometries.append(contact_point_cloud)

pen_contact_points = physxScene.get_penetrating_contact_point_positions(obj1_name, obj2_name)
print("Number of penetrating contact points: ", len(pen_contact_points))

if len(pen_contact_points) > 0:
    pen_contact_point_cloud = o3d.geometry.PointCloud()
    pen_contact_point_cloud.points = o3d.utility.Vector3dVector(pen_contact_points)
    pen_contact_point_cloud.paint_uniform_color([1, 0, 0])
    geometries.append(pen_contact_point_cloud)

hull_contact_points = physxScene.get_contact_convex_hull(obj1_name)
print("Number of hull contact points: ", len(hull_contact_points))

if len(hull_contact_points) > 0:
    hull_contact_point_cloud = o3d.geometry.PointCloud()
    hull_contact_point_cloud.points = o3d.utility.Vector3dVector(hull_contact_points)
    hull_contact_point_cloud.paint_uniform_color([0, 1, 0])
    geometries.append(hull_contact_point_cloud)

#Watchout: this can become very slow when hull_contact_point_cloud is large
stable_contact_points = physxScene.get_three_most_stable_contact_points(obj2_name)

#Create a red sphere for each stable contact point
for i in range(len(stable_contact_points)):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
    sphere.translate(stable_contact_points[i][1])
    sphere.paint_uniform_color([1, 0, 0])
    geometries.append(sphere)

#Add object geometries as linesets
for geo in object_geometries:
    #Create lineset from trianglemesh
    lineset = o3d.geometry.LineSet.create_from_triangle_mesh(geo)
    lineset.paint_uniform_color([0, 0, 0])
    geometries.append(lineset)

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
    voxels.append(obj2_voxel_centres)

    #Check if the centres from the first object are inside the voxel grid of the second object
    obj1_included_indices = obj1_voxelgrid.check_if_included(o3d.utility.Vector3dVector(obj2_voxel_positions))
    #Check if the centres from the second object are inside the voxel grid of the first object
    obj2_included_indices = obj2_voxelgrid.check_if_included(o3d.utility.Vector3dVector(obj1_voxel_positions))


    geometries += voxels

o3d.visualization.draw_geometries(geometries)