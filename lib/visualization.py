"""
# check opengl version
sudo apt-get install mesa-utils libgl1-mesa-dri
glxinfo | grep "OpenGL version"
>> OpenGL version string: 3.1 Mesa 23.2.1-1~bpo12+rpt2
"""

import open3d as o3d
import copy
import os
import colorsys
import numpy as np

try:
    from lib.pointcloud import transform
    from lib.platform_utils import get_platform
except:
    from pointcloud import transform
    from platform_utils import get_platform


def opengl_fallback(check=True):
    # suppress OpenGL warnings
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)  # .Debug

    # disable OpenGL on Raspberry Pi
    if not check or get_platform() == 'RaspberryPi':
        use_software_rendering = '1'
    else:
        use_software_rendering = '0'
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = use_software_rendering


def __generate_colors__(n, lightness=0.5, saturation=0.8, float=False):
    start_hue, _, _ = colorsys.rgb_to_hls(1, 0.706, 0)  # yellow
    end_hue, _, _ = colorsys.rgb_to_hls(0, 0.651, 0.929)  # blue
    # print(start_hue, end_hue)

    colors = []
    for i in range(n):
        hue = start_hue + i * (end_hue - start_hue) / (n-1) if n > 1 else end_hue
        r, g, b = colorsys.hls_to_rgb(hue, lightness, saturation)
        if float:
            colors.append((r, g, b))
        else:
            colors.append((int(r * 255), int(g * 255), int(b * 255)))
    return colors


# check if object_list is a list of Open3D geometries,
# convert tensor geometries to legacy if necessary
def __validate_object_list__(object_list):
    try:
        if isinstance(object_list, o3d.geometry.Geometry):
            object_list = [object_list]

        elif isinstance(object_list, o3d.t.geometry.Geometry):
            object_list = [object_list.to_legacy()]

        elif not isinstance(object_list, list):
            raise ValueError("object_list must be a list of Open3D geometries")
        else:
            for obj in object_list:
                if isinstance(obj, o3d.geometry.Geometry):
                    pass
                elif isinstance(obj, o3d.t.geometry.Geometry):
                    obj = obj.to_legacy()
                else:
                    raise ValueError("All elements in object_list must be Open3D geometries")
        return object_list
    except ValueError as e:
        print(e)


def visualize(object_list, title="PiLiDAR", transformation=None, fullscreen=True, size=(1280,720), view="front", point_size=1,
              unlit=False, backface=True, point_colors="color", bgcolor=(0.15, 0.15, 0.15), enable_fallback=True):

    object_list = __validate_object_list__(object_list)

    if enable_fallback:
        opengl_fallback()

    if point_colors.lower() == "uniform":
        point_colors = "color"
        uniform_colors = True
    else:
        uniform_colors = False

    point_color_option = {"x"       : o3d.visualization.PointColorOption.XCoordinate,
                          "y"       : o3d.visualization.PointColorOption.YCoordinate,
                          "z"       : o3d.visualization.PointColorOption.ZCoordinate,
                          "normal"  : o3d.visualization.PointColorOption.Normal,
                          "color"   : o3d.visualization.PointColorOption.Color
                          }[point_colors.lower()]

    # Deepcopy to avoid modifying the original data
    if transformation is not None or point_color_option == 5:
        object_list = copy.deepcopy(object_list)

    if transformation is not None:
        # for i, object in enumerate(object_list):
        #     object_list[i] = transform(object, transformation=transformation)
        object_list[0] = transform(object_list[0], transformation=transformation)  # only transform the first object

    if uniform_colors:
        object_list = copy.deepcopy(object_list)

        colors = __generate_colors__(len(object_list), float=True)
        for i, object in enumerate(object_list):
            object_list[i].paint_uniform_color(colors[i])

    views = {"top":     {"zoom": 0.1,   "front": (0, 0, 1),     "lookat": (0, 0, 0),    "up": (0, 1, 0)},
             "front":   {"zoom": 0.1,   "front": (0, -1, 0),    "lookat": (0, 0, 0),    "up": (0, 0, 1)},
             "overview":{"zoom": 0.2,   "front": (-1, -1, 1),   "lookat": (0, 0, 0),    "up": (0, 0, 1)}}

    if isinstance(view, dict):
        # custom view
        v = view
    elif isinstance(view, str) and view in views.keys():
        # one of the predefined views
        v = views[view]
    else:
        print("[Error] Invalid view. defaulting to front view")
        v = views["front"]

    vis = o3d.visualization.Visualizer()

    if fullscreen:
        vis.create_window(left=0, top=0, window_name=title)
    else:
        vis.create_window(left=20, top=50, window_name=title, width=size[0], height=size[1])

    # Add the geometries to the visualizer
    for obj in object_list:
        pcd = obj.to_legacy() if isinstance(obj, o3d.t.geometry.PointCloud) else obj
        vis.add_geometry(pcd)

    # Set the view parameters
    view_control = vis.get_view_control()
    lookat = object_list[0].get_center() if v["lookat"] is None else v["lookat"]
    view_control.set_lookat(lookat)
    view_control.set_zoom(v["zoom"])
    view_control.set_front(v["front"])
    view_control.set_up(v["up"])

    # Set the background color to dark gray
    render_option = vis.get_render_option()
    render_option.background_color = np.asarray(bgcolor)
    render_option.point_size = point_size
    render_option.light_on = False if unlit else True
    render_option.mesh_show_back_face = backface
    render_option.point_color_option = point_color_option

    vis.run()
    vis.destroy_window()


def visualize_simple(object_list, uniform_colors=False, view="front", enable_fallback=True):
    if enable_fallback:
        opengl_fallback()

    if uniform_colors:
        object_list = copy.deepcopy(object_list)

        colors = __generate_colors__(len(object_list), float=True)
        for i, object in enumerate(object_list):
            object_list[i].paint_uniform_color(colors[i])


    views = {"top":   {"zoom": 0.5, "front": (0, 0, 10), "lookat": (1, 0, 0), "up": (0, -1, 0)},
             "front": {"zoom": 0.25, "front": (-1, 4, 1), "lookat": (0, 0, 0), "up": (0, 0, 1)}}

    if isinstance(view, dict):
        # custom view
        v = view
    elif isinstance(view, str) and view in views.keys():
        # predefined view
        v = views[view]
    else:
        print("[Error] Invalid view. defaulting to top view")
        v = views["top"]

    o3d.visualization.draw_geometries(object_list, mesh_show_back_face=True, zoom=v["zoom"], front=v["front"], lookat=v["lookat"], up=v["up"])


if __name__ == "__main__":
    # from pointcloud import estimate_point_normals

    # download demo data
    DemoICPPointClouds = o3d.data.DemoICPPointClouds()
    path0, path1, path2 = DemoICPPointClouds.paths

    pcd = o3d.io.read_point_cloud(path0)
    # pcd = estimate_point_normals(pcd, radius=1, max_nn=30)

    view={"zoom": 0.5, "front": (0, 0, -1), "lookat": (2, 2, 1.5), "up": (0, -1, 0)}
    visualize([pcd], view=view, point_colors="normal", fullscreen=False, title="open3d preview")
