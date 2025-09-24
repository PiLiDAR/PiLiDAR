#  HAKUNA MATATA

"""
Ransac Implementation
"""

import numpy as np
from .samplers import RandomRansacSampler


def single_fit(points, model, sampler=RandomRansacSampler,
               model_kwargs={},
               sampler_kwargs={},
               max_iterations=100,
               return_model=False,
               n_inliers_to_stop=None):
    """ RANdom SAmple Consensus for fitting model a single model to points.

    points: ndarray
        (N, M) ndarray where N is the number of points and M is the number
        scalar fields associated to each of those points.
        M is usually 3 for representing the x, y, and z coordinates of each point.

    model: Ransac_Model
        Class (NOT INSTANCE!) representing the model that will be fitted to points.
        Check ransac/models for reference.

    sampler: Ransac_Sampler
        Class (NOT INSTANCE!) used to sample points on each iteration.
        Check ransac/samplers for reference.

    model_kwargs: dict, optional
        Default: {}
        Arguments that will be used on model's instantiation.
        Variable according to passed model.

    sampler_kwargs: dict, optional
        Default: {}
        Arguments that will be used on sampler's instantiation.
        Variable according to passed sampler.

    max_iterations: int, optional
        Default: 100
        Maximum number of iterations.

    return_model: bool, optional (default False)
        Whether the best fitted model will be returned or not.

    n_inliers_to_stop: int, optional
        Default None
        If the model fits a number of inliers > n_inliers_to_stop the loop will end.

    """

    model = model(**model_kwargs)
    sampler = sampler(points, model.k, **sampler_kwargs)

    n_best_inliers = 0
    if n_inliers_to_stop is None:
        n_inliers_to_stop = len(points)

    for i in range(max_iterations):

        k_points = sampler.get_sample()

        if not model.are_valid(k_points):
            print(k_points)
            continue

        model.fit(k_points)

        all_distances = model.get_distances(points)

        inliers = all_distances <= model.max_dist

        n_inliers = np.sum(inliers)

        if n_inliers > n_best_inliers:
            n_best_inliers = n_inliers
            best_inliers = inliers

            if n_best_inliers > n_inliers_to_stop:
                break

    if return_model:
        model.least_squares_fit(points[best_inliers])
        return best_inliers, model

    else:
        return best_inliers
