import open3d as o3d
import numpy as np
import copy

def crop_geometry():
    print("1) Press 'Y' twice to align geometry with negative direction of y-axis" )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    pcd = o3d.io.read_point_cloud("cloud_bin_0.pcd")
    o3d.visualization.draw_geometries_with_editing([pcd])

#cloud compare - program do chmur punktów las

def draw_registration_results(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1,0.706, 0])
    target_temp.paint_uniform_color([0,0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def pick_points(pcd):
    print(       "1) Please pick at least three correspondences using [shift + left click]"  )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press q for close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run() #pick points
    vis.destroy_window()
    return vis.get_picked_points()



def manual_registration():
    source = o3d.io.read_point_cloud("st1.pcd")
    target = o3d.io.read_point_cloud("st2.pcd")
    draw_registration_results(source, target, np.identity(4))

    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)

    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))

    corr = np.zeros(len(picked_id_source), 2)
    corr[:,0] = picked_id_source
    corr[:,1] = picked_id_target
    print(corr)

    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint
    trans_init = p2p.compute_transformation(source, target, o3d.utility.Vector2iVector(corr))
    draw_registration_results(source, target, trans_init)

    #print(trans_init)

    threshold = 0.03
    reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
                                                          o3d.pipelines.registration.TransformationEstimationPointToPoint())

    draw_registration_results(source, target, reg_p2p.transformation)

    #print(reg_p2p.transformation)

#print(pickied_id_source)

#crop_geometry()
#manual_registration()




