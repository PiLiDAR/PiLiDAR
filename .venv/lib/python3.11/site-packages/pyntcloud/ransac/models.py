
import numpy as np
from abc import ABC, abstractmethod
from ..geometry import Plane, Sphere


class RansacModel(ABC):
    """
    Base class for ransac models.

    Parameters
    ----------
    max_dist : float
        Treshold distance to consider a point as an inlier.
    """

    def __init__(self, max_dist=1e-4):
        self.max_dist = max_dist

    def fit(self, k_points):
        return self.from_k_points(k_points)

    def get_distances(self, points):
        return self.get_projections(points, only_distances=True)

    def least_squares_fit(self, points):
        return self.from_point_cloud(points)

    @abstractmethod
    def are_valid(self, k_points):
        pass


class RansacPlane(RansacModel, Plane):

    def __init__(self, max_dist=1e-4):
        super().__init__(max_dist=max_dist)
        self.k = 3

    def are_valid(self, k_points):
        return True


class RansacSphere(RansacModel, Sphere):

    def __init__(self, max_dist=1e-4):
        super().__init__(max_dist=max_dist)
        self.k = 4

    def are_valid(self, k_points):
        # check if points are coplanar
        x = np.ones((4, 4))
        x[:-1, :] = k_points.T
        if np.linalg.det(x) == 0:
            return False
        else:
            return True
