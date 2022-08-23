import json
import math

import laspy as lp
import open3d as o3d
import numpy as np
import ezdxf
from scipy.spatial import Delaunay
from functools import reduce



def volume_under_triangle(triangle):
    p1, p2, p3 = triangle
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x3, y3, z3 = p3
    return abs((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1))*(z1+z2+z3)/6

def get_triangles_verticles(triangles, verticles):
    triangles_verticles = []
    for triangle in triangles:
        new_triangle_verticles = [verticles[triangle[0]], verticles[triangle[1]], verticles[triangle[2]]]
        triangles_verticles.append(new_triangle_verticles)
    return np.array(triangles_verticles)

point_cloud = lp.read("Wesola_halda.las")

points = np.vstack((point_cloud.x, point_cloud.y, point_cloud.z)).transpose()
colors = np.vstack((point_cloud.red, point_cloud.green, point_cloud.blue)).transpose()

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors/65535)
o3d.visualization.draw_geometries([pcd])

plane_model, inliers = pcd.segment_plane(distance_threshold = 8 , ransac_n = 160, num_iterations = 2000)

print(plane_model)
#print(inliers)

plane_pcd = pcd.select_by_index(inliers)
plane_pcd.paint_uniform_color([1,0,0])

pile_pcd = pcd.select_by_index(inliers, invert=True)
pile_pcd.paint_uniform_color([0,0,1])

o3d.visualization.draw_geometries([plane_pcd, pile_pcd])

[a, b, c, d] = plane_model

plane_pcd = plane_pcd.translate((0, 0, d / c))
stockpile_pcd = pile_pcd.translate((0, 0, d / c))

cos_theta = c / math.sqrt(a ** 2 + b ** 2 + c ** 2)
sin_theta = math.sqrt((a ** 2 + b ** 2) / (a ** 2 + b ** 2 + c ** 2))
u_1 = b / math.sqrt(a ** 2 + b ** 2)
u_2 = -a / math.sqrt(a ** 2 + b ** 2)

rotation_matrix = np.array([[cos_theta + u_1 ** 2 * (1 - cos_theta), u_1 * u_2 * (1 - cos_theta), u_2 * sin_theta],
                            [u_1 * u_2 * (1 - cos_theta), cos_theta + u_2 ** 2 * (1 - cos_theta), -u_1 * sin_theta],
                            [-u_2 * sin_theta, u_1 * sin_theta, cos_theta]])


plane_pcd.rotate(rotation_matrix)
pile_pcd.rotate(rotation_matrix)


cl, ind = plane_pcd.remove_statistical_outlier(nb_neighbors=9, std_ratio=3)
cloud_selected = plane_pcd.select_by_index(ind)
o3d.visualization.draw_geometries([cloud_selected])

downpcd = cloud_selected.voxel_down_sample(voxel_size=1)
o3d.visualization.draw_geometries([downpcd])

downpcd += plane_pcd #?

o3d.io.write_point_cloud("cloud.pts", downpcd, write_ascii=True, compressed=False)


xyz = np.asarray(downpcd.points)
xy_catalog = []
for point in xyz:
    xy_catalog.append([point[0], point[1]])

tri = Delaunay(np.array(xy_catalog))

surface = o3d.geometry.TriangleMesh()
surface.vertices = o3d.utility.Vector3dVector(xyz)
surface.triangles = o3d.utility.Vector3iVector(tri.simplices)


surface.paint_uniform_color([0,0,1])
#o3d.visualization.draw_geometries([surface], mesh_show_wireframe = True)

#print(np.asarray(surface.vertices))
#print(np.asarray(surface.triangles))

volume = reduce(lambda a, b:  a+volume_under_triangle(b), get_triangles_verticles(np.array(surface.triangles),np.array( surface.vertices)), 0)
print("Total volume = ", volume)







