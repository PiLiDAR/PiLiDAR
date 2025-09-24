try:
    import pyvista as pv
except ImportError:
    pv = None

from .common import get_colors


def plot_with_pyvista(cloud, **kwargs):
    """Plot using PyVista. Additional kwargs for controoling PyVista scene are
    listed here.


    Parameters
    ----------
    off_screen : bool, optional
        Renders off screen when False.  Useful for automated screenshots.

    notebook : bool, optional
        When True, the resulting plot is placed inline a jupyter notebook.
        Assumes a jupyter console is active.  Automatically enables off_screen.

    render_points_as_spheres : bool, optional
        Render the points as spheres

    eye_dome_lighting : bool, optional
        Leverage PyVista's Eyd Dome Lighting (EDL) shading for improved
        depth perception.

    use_panel : bool, optional
        If False, the interactive rendering from panel will not be used in
        notebooks

    cpos : list(tuple(floats))
        The camera position to use

    title : string, optional
        Title of plotting window.

    screenshot : string, optional
        The path to the PNG file to save a screenshot

    point_size : float, optional
        Alias for ``initial_point_size``
    """
    if pv is None:
        raise ImportError('PyVista must be installed to use it for plotting.')
    # Get point size
    point_size = kwargs["initial_point_size"]
    if point_size is None:
        point_size = kwargs.pop("point_size", 5.0)

    # Get an RGB array using PyntCloud
    colors = get_colors(cloud, kwargs["use_as_color"], kwargs["cmap"])

    poly_data = cloud.to_instance('pyvista', mesh=kwargs.pop("mesh", False))

    plotter = pv.Plotter(window_size=[kwargs.pop("width"), kwargs.pop("height")],
                         off_screen=kwargs.pop("off_screen", None),
                         notebook=kwargs.pop("notebook", None))

    # Add the poly data to the scene
    plotter.add_mesh(poly_data, point_size=point_size, scalars=colors, rgb=True,
                     render_points_as_spheres=kwargs.pop("render_points_as_spheres", False),
                     )

    if kwargs.pop("eye_dome_lighting", None):
        plotter.enable_eye_dome_lighting()

    # TODO: Leverage `kwargs["elev"]` and `kwargs["azim"]`

    return plotter.show(use_panel=kwargs.pop("use_panel", None),
                        title=kwargs.pop("title", None),
                        screenshot=kwargs.pop("screenshot", False),
                        cpos=kwargs.pop("cpos", None) )
