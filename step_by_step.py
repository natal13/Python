import open3d as o3d
import numpy as np
import copy

source_raw = o3d.io.read_point_cloud("cloud_bin_0.pcd")
target_raw = o3d.io.read_point_cloud("cloud_bin_1.pcd")

source = source_raw.voxel_down_sample(voxel_size = 0.02)
target = target_raw.voxel_down_sample(voxel_size = 0.02)

trans = [[0.862, 0.011, -0.507, 0.0], [-0.139, 0.967, -0.215, 0.7],
         [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]]

flip_tranform = [[1,0,0, 0], [0,0,-1,0], [0,0,-1,0], [0,0,0,1]]

source.transform(flip_tranform)
target.transform(flip_tranform)

vis = o3d.visualization.Visualizer()

vis.create_window()
vis.add_geometry(source)
vis.add_geometry(target)

threshold = 0.05
icp_iterations = 100
save_image = False


for i in range(icp_iterations):
    reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans,
                                                          o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                          o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))
    source.transform(reg_p2p.transformation)
    vis.update_geometry(source)
    vis.poll_events()
    vis.update_renderer()

    if save_image:
        vis.capture_screen_image("temp_%d04.jpg" %i)
    vis.destroy_window()






