"""
HAKUNA MATATA
"""
from .convex_hull import ConvexHull
from .delaunay import Delaunay3D
from .kdtree import KDTree
from .voxelgrid import VoxelGrid

ALL_STRUCTURES = {
    'convex_hull': ConvexHull,
    'delaunay3D': Delaunay3D,
    'kdtree': KDTree,
    'voxelgrid': VoxelGrid
}
