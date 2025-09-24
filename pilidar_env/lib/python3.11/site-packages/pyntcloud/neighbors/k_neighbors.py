
def k_neighbors(kdtree, k):
    """ Get indices of K neartest neighbors for each point

    Parameters
    ----------
    kdtree: pyntcloud.structrues.KDTree
        The KDTree built on top of the points in point cloud

    k: int
        Number of neighbors to find

    Returns
    -------
    k_neighbors: (N, k) array
        Where N = kdtree.data.shape[0]
    """
    # [1] to select indices and ignore distances
    # [:,1:] to discard self-neighbor
    return kdtree.query(kdtree.data, k=k + 1, workers=-1)[1][:, 1:]
