import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from lib.pointcloud import load_pointcloud, save_pointcloud, get_transform_vectors, transform
from lib.registration import global_registration, icp, color_icp  # noqa: F401 (used in commented-out block)
from lib.visualization import visualize


def run_registration(source, target, voxel_size=0.15, fast_gr=False, p2l_icp=True, show=False):

    # GLOBAL REGISTRATION
    reg_gr = global_registration(source, target, voxel_size, fast=fast_gr,
                                 normals_nn=30, fpfh_nn=100, max_iteration=100000,
                                 confidence=0.999, edgelength=0.9)

    if show:
        visualize([source, target], transformation=reg_gr.transformation,
                point_colors="uniform", view=view, point_size=point_size)

    # # evaluate with voxel size of ICP
    # evaluation = o3d.pipelines.registration.evaluate_registration(source, target, voxel_size*0.4, reg_gr.transformation)
    # print("GR:", evaluation)


    # ICP REGISTRATION
    reg_icp = icp(source, target, reg_gr.transformation,
                  voxel_size, p2l=p2l_icp)

    if show:
        visualize([source, target], transformation=reg_icp.transformation,
                point_colors="uniform", view=view, point_size=point_size)

    # return the FINE (ICP) transform — the global one is only its seed
    return reg_icp.transformation


def get_bounding_box_size(pcd):
    bbox = pcd.get_axis_aligned_bounding_box()
    return bbox.get_extent()


if __name__ == "__main__":

    view={"zoom": 0.1, "front": (-1, 1, 0.5), "lookat": (0, 0, 0), "up": (0, 0, 1)}
    point_size = 1.0
    voxel_size = 0.15  # meter

    pcd_list= [load_pointcloud(str(ROOT / "scans" / "261015-1839" / "261015-1839_half_left.ply")),
               load_pointcloud(str(ROOT / "scans" / "261015-1839" / "261015-1839_half_right.ply"))]

    source = pcd_list[0]
    target = pcd_list[1]

    # # bounding box size
    print(f"Bounding box: {get_bounding_box_size(source)}")

    # VISUALIZE BEFORE
    visualize([source, target], point_colors="uniform", view=view, point_size=point_size)


    transformation = run_registration(source, target, voxel_size=voxel_size, show=False)

    # Translation and rotation vectors
    icp_translation, icp_euler = get_transform_vectors(transformation)
    print(f"[ICP]\t translate:\t{icp_translation} \trotate:\t{icp_euler})")

    # VISUALIZE
    visualize([source, target], transformation=transformation,
        point_colors="uniform", view=view, point_size=point_size)

    # EXPORT TRANSFORMED POINT CLOUDS
    transformed_source = transform(source, transformation=transformation)
    save_pointcloud(transformed_source + target, str(ROOT / "scans" / "registration_test.ply"))


    # # ------------------------------------------------
    # # COLORED POINT CLOUD REGISTRATION

    # visualize([source, target], view=view, transformation=transformation, point_size=point_size)

    # reg_color = color_icp(source, target,transformation)
    # #print(result_icp)
    # visualize([source, target], transformation=reg_color.transformation,
    #           point_colors="uniform", view=view, point_size=point_size)
