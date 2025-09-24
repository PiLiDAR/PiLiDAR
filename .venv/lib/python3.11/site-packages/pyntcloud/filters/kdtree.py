import numpy as np

from scipy.stats import zscore

from .base import Filter


class KDTreeFilter(Filter):
    def __init__(self, *, pyntcloud, kdtree_id):
        """
        Parameters
        ----------
        pyntcloud: pyntcloud.PyntCloud
        kdtree_id: pyntcloud.structures.KDTree.id
            Usually returned from PyntCloud.add_structure("kdtree"):
            kdtree_id = my_cloud.add_structure("kdtree")
        """
        super().__init__(pyntcloud=pyntcloud)
        self.kdtree_id = kdtree_id

    def extract_info(self):
        self.points = self.pyntcloud.xyz
        self.kdtree = self.pyntcloud.structures[self.kdtree_id]

    def compute(self):
        pass


class RadiusOutlierRemovalFilter(KDTreeFilter):
    """Compute a Radius Outlier Removal filter using the given KDTree.

    Parameters
    ----------
    kdtree_id: pyntcloud.structures.KDTree.id
    k : int
        Number of neighbors that will be used to compute the filter.
    r : float
        The radius of the sphere with center on each point. The filter
        will look for the required number of neighboors inside that sphere.

    Notes
    -----
    > The distances between each point and its 'k' nearest neighbors that
        exceed the given 'r' are marked as False.

    > The points having any distance marked as False will be trimmed.

    The parameter r should be used in order to adjust the filter to
    the desired result.

    A LOWER 'r' value will result in a HIGHER number of points trimmed.
    """

    def __init__(self, *, pyntcloud, kdtree_id, k, r):
        super().__init__(pyntcloud=pyntcloud, kdtree_id=kdtree_id)
        self.k = k
        self.r = r

    def compute(self):
        distances = self.kdtree.query(self.points, k=self.k, workers=-1)[0]
        ror_filter = np.all(distances < self.r, axis=1)

        return ror_filter


class StatisticalOutlierRemovalFilter(KDTreeFilter):
    """Compute a Statistical Outlier Removal filter using the given KDTree.

    Parameters
    ----------
    kdtree_id: pyntcloud.structures.KDTree.id
    k : int
        Number of neighbors that will be used to compute the filter.
    z_max: float
        The maximum Z score which determines if the point is an outlier.

    Notes
    -----
    > For each point, the mean of the distances between it and its 'k' nearest
        neighbors is computed.

    > The Z score of those means is computed.

    > Points with a Z score outside the range [-z_max, z_max] are marked
        as False, in order to be trimmed.

    The optional parameter z_max should be used in order to adjust
    the filter to the desired result.

    A LOWER 'z_max' value will result in a HIGHER number of points trimmed.
    """

    def __init__(self, *, pyntcloud, kdtree_id, k, z_max):
        super().__init__(pyntcloud=pyntcloud, kdtree_id=kdtree_id)
        self.k = k
        self.z_max = z_max

    def compute(self):
        distances = self.kdtree.query(self.points, k=self.k, workers=-1)[0]
        z_distances = zscore(np.mean(distances, axis=1), ddof=1)
        sor_filter = abs(z_distances) < self.z_max

        return sor_filter
