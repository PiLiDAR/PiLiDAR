from scipy.spatial import KDTree as sKDTree

from .base import Structure


class KDTree(sKDTree, Structure):

    def __init__(self, *, points, leafsize=16, compact_nodes=False, balanced_tree=False):
        Structure.__init__(self, points=points)
        self._leafsize = leafsize
        self._compact_nodes = compact_nodes
        self._balanced_tree = balanced_tree

    def compute(self):
        self.id = "K({},{},{})".format(self._leafsize, self._compact_nodes, self._balanced_tree)
        super(sKDTree, self).__init__(
            self._points,
            leafsize=self._leafsize,
            compact_nodes=self._compact_nodes,
            balanced_tree=self._balanced_tree)
