import numpy as np


def r_neighbors(kdtree, r):
    """ Get indices of all neartest neighbors with a distance < r for each point

    Parameters
    ----------
    kdtree: pyntcloud.structrues.KDTree
        The KDTree built on top of the points in point cloud

    r: float
        Maximum distance to consider a neighbor

    Returns
    -------
    r_neighbors: (N, X) ndarray of lists
        Where N = kdtree.data.shape[0]
        len(X) varies for each point
    """
    return np.array(kdtree.query_ball_tree(kdtree, r))
