import sys
import numpy as np
import open3d as o3d

filename = '/home/natu/gitproj/3dpcp_book_codes/3rdparty/Open3D/examples/test_data/bathtub_0154.ply'

print("Load from ", filename)

pcd = o3d.io.read_triangle_mesh(filename)
o3d.visualization.draw_geometries([pcd])
