import numpy as np

from .base import ScalarField
from ..geometry.coord_systems import (
    cartesian_to_spherical,
    cartesian_to_cylindrical)
from ..ransac import (
    single_fit,
    RANSAC_MODELS,
    RANSAC_SAMPLERS)


class XYZScalarField(ScalarField):
    def extract_info(self):
        self.points = self.pyntcloud.xyz


class PlaneFit(XYZScalarField):
    """
    Get inliers of the best RansacPlane found.
    """

    def __init__(self, *, pyntcloud, max_dist=1e-4, max_iterations=100, n_inliers_to_stop=None):
        self.model = RANSAC_MODELS["plane"]
        self.sampler = RANSAC_SAMPLERS["random"]
        self.name = "is_plane"
        self.model_kwargs = {"max_dist": max_dist}
        self.max_iterations = max_iterations
        self.n_inliers_to_stop = n_inliers_to_stop

        super().__init__(pyntcloud=pyntcloud)

    def compute(self):
        inliers = single_fit(self.points, self.model, self.sampler,
                             model_kwargs=self.model_kwargs,
                             max_iterations=self.max_iterations,
                             n_inliers_to_stop=self.n_inliers_to_stop)
        self.to_be_added[self.name] = inliers.astype(np.uint8)


class SphereFit(XYZScalarField):
    """
    Get inliers of the best RansacSphere found.
    """

    def __init__(self, *, pyntcloud, max_dist=1e-4, max_iterations=100, n_inliers_to_stop=None):
        super().__init__(pyntcloud=pyntcloud)
        self.model = RANSAC_MODELS["sphere"]
        self.sampler = RANSAC_SAMPLERS["random"]
        self.name = "is_sphere"
        self.model_kwargs = {"max_dist": max_dist}
        self.max_iterations = max_iterations
        self.n_inliers_to_stop = n_inliers_to_stop

    def compute(self):
        inliers = single_fit(self.points, self.model, self.sampler,
                             model_kwargs=self.model_kwargs,
                             max_iterations=self.max_iterations,
                             n_inliers_to_stop=self.n_inliers_to_stop)
        self.to_be_added[self.name] = inliers.astype(np.uint8)


class CustomFit(XYZScalarField):
    """
    Get inliers of the best custom model found.
    """

    def __init__(self, pyntcloud, model, sampler, name, model_kwargs={},
                 sampler_kwargs={}, max_iterations=100, n_inliers_to_stop=None):
        super().__init__(pyntcloud=pyntcloud)
        self.model = model
        self.sampler = sampler
        self.name = name
        self.model_kwargs = model_kwargs
        self.sampler_kwargs = sampler_kwargs
        self.max_iterations = max_iterations
        self.n_inliers_to_stop = n_inliers_to_stop

    def compute(self):
        inliers = single_fit(self.points, self.model, self.sampler,
                             model_kwargs=self.model_kwargs,
                             max_iterations=self.max_iterations,
                             n_inliers_to_stop=self.n_inliers_to_stop)
        self.to_be_added[self.name] = inliers.astype(np.uint8)


class SphericalCoordinates(XYZScalarField):
    """
    Get radial, azimuthal and polar values.
    """

    def __init__(self, *, pyntcloud, degrees=True):
        super().__init__(pyntcloud=pyntcloud)
        self.degrees = degrees

    def compute(self):
        radial, polar, azimuthal = cartesian_to_spherical(
            self.points, degrees=self.degrees)

        self.to_be_added["radial"] = radial
        self.to_be_added["polar"] = polar
        self.to_be_added["azimuthal"] = azimuthal


class CylindricalCoordinates(XYZScalarField):
    """
    Get ro and phi values.
    The z value in cylindrical coordinates remain unchanged.
    """

    def __init__(self, *, pyntcloud, degrees=True):
        self.degrees = degrees
        super().__init__(pyntcloud=pyntcloud)

    def compute(self):
        radial_cylindrical, angular_cylindrical, z = cartesian_to_cylindrical(
            self.points, degrees=self.degrees)

        self.to_be_added["radial_cylindrical"] = radial_cylindrical
        self.to_be_added["angular_cylindrical"] = angular_cylindrical
