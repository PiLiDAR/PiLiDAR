import numpy as np
from .base import Filter


class XYZFilter(Filter):
    def extract_info(self):
        self.points = self.pyntcloud.xyz


class BoundingBoxFilter(XYZFilter):
    """
    Compute a bounding box filter for the given points.

    Parameters
    ----------

    min_x, max_x, min_y, max_y, min_z, max_z: float
        The bounding box limits for each coordinate.
        If some limits are missing, the default values are -infinite
        for the min_i and infinite for the max_i.

    """

    def __init__(self, *, pyntcloud, min_x=-np.inf, max_x=np.inf, min_y=-np.inf,
                 max_y=np.inf, min_z=-np.inf, max_z=np.inf):
        super().__init__(pyntcloud=pyntcloud)
        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.min_z, self.max_z = min_z, max_z

    def compute(self):

        bound_x = np.logical_and(self.points[:, 0] > self.min_x,
                                 self.points[:, 0] < self.max_x)
        bound_y = np.logical_and(self.points[:, 1] > self.min_y,
                                 self.points[:, 1] < self.max_y)
        bound_z = np.logical_and(self.points[:, 2] > self.min_z,
                                 self.points[:, 2] < self.max_z)

        bb_filter = np.logical_and(np.logical_and(bound_x, bound_y), bound_z)

        return bb_filter
