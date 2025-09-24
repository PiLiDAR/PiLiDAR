from .base import Sampler

import numpy as np
import pandas as pd


class PointsSampler(Sampler):
    """
    """
    def extract_info(self):
        self.points = self.pyntcloud.points


class RandomPointsSampler(PointsSampler):
    """
    Parameters
    ----------
    n: int
        Number of unique points that will be chosen.
    """

    def __init__(self, *, pyntcloud, n):
        super().__init__(pyntcloud=pyntcloud)
        self.n = n

    def compute(self):
        if self.n > len(self.points):
            raise ValueError("n can't be higher than the number of points in the PyntCloud.")
        return self.points.sample(self.n).reset_index(drop=True)


class FarthestPointsSampler(PointsSampler):

    """
    Parameters
    ----------
    n: int
        Number of unique points that will be chosen.
    d_metric: 3*3 numpy array
        a positive semi-definite matrix which defines a distance metric
    """

    def __init__(self, *, pyntcloud, n, d_metric=np.eye(3)):
        """d_metric -> Euclidean distance space by default, can be modified to other Mahalanobis distance as well"""
        super().__init__(pyntcloud=pyntcloud)
        self.n = n
        if not np.all(np.linalg.eigvals(d_metric) >= 0):
            raise ValueError("the distance metric must be positive semi-definite")
        self.d_metric = d_metric

    def cal_distance(self, point, solution_set):
        """
        :param point: points which is not sampled yet, N*10 or N*3 numpy array
        :param solution_set: the points which has been selected, M*3 or M*10 array
        :return: a (N, ) array, where each element is equal to the sum of distance
        of all points in 'solution_set' w.r.t the unselected point in the 'point'
        """
        distance_sum = np.zeros(len(point))

        for pt in solution_set:
            distance_sum += np.diag(np.dot((point[:, :3]-pt[:3]), self.d_metric@(point[:, :3]-pt[:3]).T))
        return distance_sum

    def compute(self):
        "incremental farthest search"
        if self.n > len(self.points):
            raise ValueError("sampled points can't be more than the original input")
        remaining_points = self.points.values

        # the sampled points set as the return
        select_idx = np.random.randint(low=0, high=len(self.points))
        # to remain the shape as (1, n) instead of (n, )
        solution_set = remaining_points[select_idx: select_idx+1]
        remaining_points = np.delete(remaining_points, select_idx, 0)

        for _ in range(self.n - 1):
            distance_sum = self.cal_distance(remaining_points, solution_set)
            select_idx = np.argmax(distance_sum)
            solution_set = np.concatenate([solution_set, remaining_points[select_idx:select_idx+1]], axis=0)
            remaining_points = np.delete(remaining_points, select_idx, 0)

        return pd.DataFrame(solution_set, columns=self.points.columns)
