import numpy as np
import open3d as o3d
import copy

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

R = o3d.geometry.get_rotation_matrix_from_yxz([np.pi/3, 0, 0])
print("R:", R)
R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.pi/3, 0])
print("R:", R)
R = o3d.geometry.get_rotation_matrix_from_quaternion([np.cos(np.pi/6),0 , np.sin(np.pi/6), 0])
print("R:", R)

mesh_r = copy.deepcopy(mesh)
mesh_r.rotate(R, center=[0,0,0])

t = [0.2, 1.7, 0]
mesh_t = copy.deepcopy(mesh_r).translate(t)
print("type q to continue")
o3d.visualization.draw_geometries([mesh, mesh_t])

T = np.eye(4)
T[:3, :3] = R
T[:3,3] = t
mesh_t = copy.deepcopy(mesh).transform(T)
o3d.visualization.draw_geometries([mesh, mesh_t])

mesh_s = copy.deepcopy(mesh_t)
mesh_s.scale(0.5, center=mesh_s.get_center())
o3d.visualization.draw_geometries([mesh, mesh_s])