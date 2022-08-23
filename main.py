import json

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

point_cloud = lp.read("buk.las")

points = np.vstack((point_cloud.x, point_cloud.y, point_cloud.z)).transpose()
colors = np.vstack((point_cloud.red, point_cloud.green, point_cloud.blue)).transpose()

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors/65535)
#o3d.visualization.draw_geometries([pcd])

xm = min(points[:, 0])
ym = min(points[:, 1])
#print(xm, ym)

moveVector = np.ndarray(3, buffer = np.array([-xm+1000, -ym+1000, 0]))
pcd.translate(moveVector)
#o3d.visualization.draw_geometries([pcd])
polylines = ezdxf.readfile("obwiednie.dxf").query("POLYLINE")
#print(polylines)

polyline = polylines[0]
converted_points = []
for point in polyline.points():
    converted_points.append([point[0]+moveVector[0], point[1]+moveVector[1], point[2]])

print(converted_points)

my_json = {}
my_json['axis_max'] = 1000
my_json['axis_min'] = 0
my_json['bounding_polygon'] = converted_points
my_json['class_name'] = "SelectionPolygonVolume"
my_json['orthogonal_axis'] = "Z"
my_json['version_major'] = 1
my_json['version_minor'] = 0

with open(polyline.dxf.layer + ".json", "w") as outfile:
    json.dump(my_json, outfile, indent=4)

vol_selected = o3d.visualization.read_selection_polygon_volume(polyline.dxf.layer + ".json")

polygon_list = []
polygon_list.append(converted_points)

ref = o3d.geometry.PointCloud()
ref.points = o3d.utility.Vector3dVector(polygon_list[0])

cloud_selected = vol_selected.crop_point_cloud(pcd)

#o3d.visualization.draw_geometries([cloud_selected])

cl, ind = cloud_selected.remove_statistical_outlier(nb_neighbors=30, std_ratio=2)
cloud_selected = cloud_selected.select_by_index(ind)
#o3d.visualization.draw_geometries([cloud_selected])

downpcd = cloud_selected.voxel_down_sample(voxel_size=1)
#o3d.visualization.draw_geometries([downpcd])

downpcd += ref

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
o3d.visualization.draw_geometries([surface], mesh_show_wireframe = True)

#print(np.asarray(surface.vertices))
#print(np.asarray(surface.triangles))

volume = reduce(lambda a, b:  a+volume_under_triangle(b), get_triangles_verticles(np.array(surface.triangles),np.array( surface.vertices)), 0)
print("Total volume = ", volume)


