from .voxelgrid import plot_voxelgrid_with_threejs

DESCRIPTION = """\
PyntCloud
{} points with {} scalar fields
{} faces in mesh
{} kdtrees
{} voxelgrids
Centroid: {}, {}, {}
Other attributes:{}
"""

AVAILABLE_BACKENDS = []
# Add each backend in order of preference
# Add pythreejs
try:
    import pythreejs
    AVAILABLE_BACKENDS.append("pythreejs")
except ImportError:
    pass
# Add PyVista
try:
    import pyvista
    AVAILABLE_BACKENDS.append("pyvista")
except ImportError:
    pass
# Add matplotlib
try:
    import matplotlib
    AVAILABLE_BACKENDS.append("matplotlib")
except ImportError:
    pass
# Add threejs
try:
    from IPython.display import IFrame
    AVAILABLE_BACKENDS.append("threejs")
except ImportError:
    pass
