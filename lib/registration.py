'''
Transformations
http://www.open3d.org/docs/release/tutorial/visualization/non_blocking_visualization.html
http://www.open3d.org/docs/latest/tutorial/Basic/transformation.html

Global Registration
https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html

ICP registration
https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html

Colored registration
https://www.open3d.org/docs/release/tutorial/pipelines/colored_pointcloud_registration.html
'''

import open3d as o3d
import time

try:
    from lib.pointcloud import estimate_point_normals
    from lib.platform_utils import get_platform
except:
    from pointcloud import estimate_point_normals
    from platform_utils import get_platform

platform = get_platform()


# Global Registration using RANSAC
def global_registration(source, target, voxel_size, fast=False, normals_nn=30, fpfh_nn=100, max_iteration=100000, confidence=0.999, edgelength=0.9):

    def compute_fpfh(pcd_down, voxel_size, fpfh_nn=100):
        radius_feature = voxel_size * 5
        #print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        KD_search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=fpfh_nn)
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down, KD_search_param)
        return pcd_fpfh

    source_down = estimate_point_normals(source.voxel_down_sample(voxel_size), radius=voxel_size*2, max_nn=normals_nn)
    source_fpfh = compute_fpfh(source_down, voxel_size, fpfh_nn)

    target_down = estimate_point_normals(target.voxel_down_sample(voxel_size), radius=voxel_size*2, max_nn=normals_nn)
    target_fpfh = compute_fpfh(target_down, voxel_size, fpfh_nn)

    if fast:
        dist_thres = voxel_size * 0.5
        gr_result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=dist_thres))

    else: # RANSAC
        dist_thres = voxel_size * 1.5
        gr_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True, dist_thres,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(edgelength),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist_thres)],
                o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration, confidence))

    return gr_result


# ICP (P2P or P2L) Registration
def icp(source, target, transform, voxel_size, p2l=True, p2p_max_iteration=2000):
    dist_thres = voxel_size * 0.4

    if p2l:  # point-to-plane ICP
        method = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        reg_icp = o3d.pipelines.registration.registration_icp(source, target, dist_thres, transform, method)

    else:  # point-to-point ICP
        method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        reg_icp = o3d.pipelines.registration.registration_icp(
            source, target, dist_thres, transform, method,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=p2p_max_iteration))

    return reg_icp

# COLORED POINT CLOUD REGISTRATION
def color_icp(source, target, transform, normals_nn=30):
    voxel_radius = [0.1, 0.04, 0.02, 0.01]
    max_iter = [100, 50, 30, 14]
    current_transformation = transform

    for scale in range(4):
        iter = max_iter[scale]
        radius = voxel_radius[scale]

        # Downsample
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        # Estimate normals
        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=normals_nn))
        target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=normals_nn))

        # colored point cloud registration
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                            relative_rmse=1e-6,
                                                            max_iteration=iter))
        current_transformation = result_icp.transformation

    return result_icp


if __name__ == "__main__":
    import numpy as np
    from pointcloud import save_pointcloud, get_transform_vectors, transform
    from visualization import visualize


    def prepare_dataset():
        demo_icp_pcds = o3d.data.DemoICPPointClouds()
        source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
        target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])
        trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        source.transform(trans_init)
        return source, target


    view={"zoom": 0.5, "front": (0, 0, -1), "lookat": (2, 2, 1.5), "up": (0, -1, 0)}
    point_size = 3

    source, target = prepare_dataset()
    if platform == 'Windows':
        visualize([source, target], point_colors="uniform", view=view, point_size=point_size)


    # ------------------------------------------------
    # GLOBAL REGISTRATION
    voxel_size = 0.05  # means 5cm for this dataset
    fast_gr = False
    p2l_icp = True

    start = time.time()
    reg_gr = global_registration(source, target, voxel_size, fast=fast_gr)
    print("GR duration:", time.time() - start)

    if platform == 'Windows':
        visualize([source, target], transformation=reg_gr.transformation, point_colors="uniform", view=view, point_size=point_size)


    ransac_translation, ransac_euler = get_transform_vectors(reg_gr.transformation)
    print(f"[RANSAC] translate:\t{ransac_translation})")
    print(f"[RANSAC] rotate:\t{ransac_euler})")


    # evaluate with voxel size of ICP
    evaluation = o3d.pipelines.registration.evaluate_registration(source, target, voxel_size*0.4, reg_gr.transformation)
    print("GR:", evaluation)


    # ------------------------------------------------
    # ICP REGISTRATION
    start = time.time()
    reg_icp = icp(source, target, reg_gr.transformation, voxel_size, p2l=p2l_icp)
    print("ICP duration:", time.time() - start)

    print("ICP:", reg_icp)

    if platform == 'Windows':
        visualize([source, target], transformation=reg_icp.transformation, point_colors="uniform", view=view, point_size=point_size)


    # EXPORT TRANSFORMED POINT CLOUDS
    transformed_source = transform(source, transformation=reg_icp.transformation)
    save_pointcloud(transformed_source + target, "scans/registration_test.ply")


    # ------------------------------------------------
    # COLORED POINT CLOUD REGISTRATION
    if platform == 'Windows':
        visualize([source, target], view=view, transformation=reg_gr.transformation, point_size=point_size)

    reg_color = color_icp(source, target,reg_gr.transformation)
    #print(result_icp)

    if platform == 'Windows':
        visualize([source, target], transformation=reg_color.transformation, view=view, point_size=point_size)
