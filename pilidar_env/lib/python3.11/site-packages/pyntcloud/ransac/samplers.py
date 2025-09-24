import numpy as np

from abc import ABC, abstractmethod
from ..structures import VoxelGrid


class RansacSampler(ABC):
    """ Base class for ransac samplers.

    Parameters
    ----------
    points : ndarray
        (N, M) ndarray where N is the number of points and M is the number
        scalar fields associated to each of those points.
        M is usually 3 for representing the x, y, and z coordinates of each point.

    k : int
        The number of points that will be sampled in each call of get_sample().
        This number depends on the model used. See ransac/models.py.
    """

    def __init__(self, points, k):
        self.points = points
        self.k = k

    @abstractmethod
    def get_sample(self):
        pass


class RandomRansacSampler(RansacSampler):
    """ Sample random points.

    Inherits from RansacSampler.

    """

    def __init__(self, points, k):
        super().__init__(points, k)

    def get_sample(self):
        """ Get k unique random points.
        """
        sample = np.random.choice(len(self.points), self.k, replace=False)
        return self.points[sample]


class VoxelgridRansacSampler(RansacSampler):
    """ Sample random points inside the same random voxel.

    Inherits from RansacSampler.

    Parameters
    ----------
    points: (N, 3) numpy.array
    k: int
        Numbber of points to sample.
    n_x, n_y, n_z :  int, optional
        Default: 1
        The number of segments in which each axis will be divided.
        Ignored if corresponding size_x, size_y or size_z is not None.
    size_x, size_y, size_z : float, optional
        Default: None
        The desired voxel size along each axis.
        If not None, the corresponding n_x, n_y or n_z will be ignored.
    regular_bounding_box : bool, optional
        Default: True
        If True, the bounding box of the point cloud will be adjusted
        in order to have all the dimensions of equal length.
    """

    def __init__(self,
                 points, k, n_x=1, n_y=1, n_z=1, size_x=None, size_y=None, size_z=None, regular_bounding_box=True):
        super().__init__(points, k)
        self.voxelgrid = VoxelGrid(
            points=self.points, n_x=n_x, n_y=n_y, n_z=n_z, size_x=size_x, size_y=size_y, size_z=size_z,
            regular_bounding_box=regular_bounding_box)
        self.voxelgrid.compute()

    def get_sample(self):
        """ Get k unique random points from the same voxel of one randomly picked point.
        """
        # pick one point and get its voxel index
        idx = np.random.randint(0, len(self.points))
        voxel = self.voxelgrid.voxel_n[idx]

        # get index of points inside that voxel and convert to probabilities
        points_in_voxel = (self.voxelgrid.voxel_n == voxel).astype(int)
        points_in_voxel = points_in_voxel / points_in_voxel.sum()

        sample = np.random.choice(
            len(self.points), self.k, replace=False, p=points_in_voxel)

        return self.points[sample]
