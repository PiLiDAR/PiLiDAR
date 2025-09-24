try:
    import ipywidgets
except ImportError:
    ipywidgets = None
try:
    import pythreejs
except ImportError:
    pythreejs = None
try:
    from IPython.display import display
except ImportError:
    display = None

from .common import get_colors


def get_pointcloud_pythreejs(xyz, colors):
    points_geometry = pythreejs.BufferGeometry(
        attributes=dict(
            position=pythreejs.BufferAttribute(xyz, normalized=False),
            color=pythreejs.BufferAttribute(list(map(tuple, colors)))))

    points_material = pythreejs.PointsMaterial(
        vertexColors='VertexColors')

    return pythreejs.Points(
        geometry=points_geometry,
        material=points_material)


def get_polylines_pythreejs(polylines):
    lines = []
    for x in polylines:
        line_geometry = pythreejs.Geometry(
            vertices=x["vertices"])
        linewidth = x.get("linewidth", 1.0)
        line = pythreejs.Line(
            geometry=line_geometry,
            material=pythreejs.LineBasicMaterial(color=x["color"], linewidth=linewidth),
            type='LinePieces')
        lines.append(line)

    return lines


def get_camera_pythreejs(centroid, xyz, width, height):
    camera = pythreejs.PerspectiveCamera(
        fov=90,
        aspect=width / height,
        position=tuple(centroid + [0, abs(xyz.max(0)[1]), abs(xyz.max(0)[2]) * 1.5]),
        up=[0, 0, 1])
    camera.lookAt(tuple(centroid))
    return camera


def get_orbit_controls(camera, centroid):
    orbit_controls = pythreejs.OrbitControls(controlling=camera)
    orbit_controls.target = tuple(centroid)
    return orbit_controls


def plot_with_pythreejs(cloud, **kwargs):
    if ipywidgets is None:
        raise ImportError("ipywidgets is needed for plotting with pythreejs backend.")
    if pythreejs is None:
        raise ImportError("pythreejs is needed for plotting with pythreejs backend.")
    if display is None:
        raise ImportError("IPython is needed for plotting with pythreejs backend.")

    colors = get_colors(cloud, kwargs["use_as_color"], kwargs["cmap"])

    ptp = cloud.xyz.ptp()

    children = []
    widgets = []

    if kwargs["mesh"]:
        raise NotImplementedError("Plotting mesh geometry with pythreejs backend is not supported yet.")

    if kwargs["polylines"]:
        lines = get_polylines_pythreejs(kwargs["polylines"])
        children.extend(lines)

    points = get_pointcloud_pythreejs(cloud.xyz, colors)
    children.append(points)

    initial_point_size = kwargs["initial_point_size"] or ptp / 10
    size = ipywidgets.FloatSlider(
        value=initial_point_size,
        min=0.0,
        max=initial_point_size * 10,
        step=initial_point_size / 100)
    ipywidgets.jslink((size, 'value'), (points.material, 'size'))
    widgets.append(ipywidgets.Label('Point size:'))
    widgets.append(size)

    if kwargs["scene"]:
        kwargs["scene"].children = [points] + list(kwargs["scene"].children)
    else:
        camera = get_camera_pythreejs(cloud.centroid, cloud.xyz, kwargs["width"], kwargs["height"])
        children.append(camera)

        controls = [get_orbit_controls(camera, cloud.centroid)]

        scene = pythreejs.Scene(children=children)

        renderer = pythreejs.Renderer(
            scene=scene,
            camera=camera,
            controls=controls,
            width=kwargs["width"],
            height=kwargs["height"])

        display(renderer)

        color = ipywidgets.ColorPicker(value=kwargs["background"])
        ipywidgets.jslink((color, 'value'), (scene, 'background'))
        widgets.append(ipywidgets.Label('Background color:'))
        widgets.append(color)

    display(ipywidgets.HBox(children=widgets))

    return scene if kwargs["return_scene"] else None
