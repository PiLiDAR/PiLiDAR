import pandas as pd
from scipy.spatial import Delaunay
from itertools import combinations

from .base import Structure


class Delaunay3D(Delaunay, Structure):

    def __init__(self, points,
                 furthest_site=False,
                 incremental=False,
                 qhull_options=None):
        Structure.__init__(self, points=points)
        self._furthest_site = furthest_site
        self._incremental = incremental
        self._qhull_options = qhull_options

    def compute(self):
        """ABC API"""
        self.id = "D({},{})".format(self._furthest_site, self._qhull_options)
        Delaunay.__init__(self,
                          self._points,
                          self._furthest_site,
                          self._incremental,
                          self._qhull_options)

    def get_mesh(self):
        """
        Decompose the tetrahedrons into triangles to build mesh.

        The returned mesh is in mesh-vertex format, suitable for
        been assigned to PyntCloud.mesh.
        """
        triangles = []
        for tetra in self.simplices:
            for tri in combinations(tetra, 3):
                triangles.append([tri[0], tri[1], tri[2]])
        mesh = pd.DataFrame(triangles, columns=["v1", "v2", "v3"])

        return mesh
