import numpy as np
from .base import ScalarField


class NormalsScalarField(ScalarField):
    def extract_info(self):
        self.normals = self.pyntcloud.points[["nx", "ny", "nz"]].values


class InclinationDegrees(NormalsScalarField):
    """ Vertical inclination with respect to Z axis in degrees.
    """
    def compute(self):
        inclination = np.arccos(self.normals[:, -1])
        self.to_be_added["inclination_deg"] = np.rad2deg(inclination)


class InclinationRadians(NormalsScalarField):
    """ Vertical inclination with respect to Z axis in radians.
    """
    def compute(self):
        inclination = np.arccos(self.normals[:, -1])
        self.to_be_added["inclination_rad"] = inclination


class OrientationDegrees(NormalsScalarField):
    """ Horizontal orientation with respect to the XY plane in degrees.
    """
    def compute(self):
        angle = np.arctan2(self.normals[:, 0], self.normals[:, 1])
        # convert (-180 , 180) to (0 , 360)
        angle = np.where(angle < 0, angle + (2 * np.pi), angle)
        self.to_be_added["orientation_deg"] = np.rad2deg(angle)


class OrientationRadians(NormalsScalarField):
    """ Horizontal orientation with respect to the XY plane in radians.
    """
    def compute(self):
        angle = np.arctan2(self.normals[:, 0], self.normals[:, 1])
        # convert (-180 , 180) to (0 , 360)
        angle = np.where(angle < 0, angle + (2 * np.pi), angle)
        self.to_be_added["orientation_rad"] = angle
