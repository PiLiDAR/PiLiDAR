import numpy as np
from .base import ScalarField


class EigenValuesScalarField(ScalarField):
    """
    Parameters
    ----------
    ev : list of str
        Column names of the eigen values.
        Tip:
            ev = self.add_scalar_field("eigen_values", ...)
    """

    def __init__(self, *, pyntcloud, ev):
        super().__init__(pyntcloud=pyntcloud)
        self.k = ev[0].split("e1")[1]
        self.ev = ev

    def extract_info(self):
        self.ev = self.pyntcloud.points[self.ev].values


class Anisotropy(EigenValuesScalarField):
    """
    """
    def compute(self):
        name = "anisotropy{}".format(self.k)
        ev = self.ev
        self.to_be_added[name] = np.nan_to_num((ev[:, 0] - ev[:, 2]) / ev[:, 0])


class Curvature(EigenValuesScalarField):
    """
    """
    def compute(self):
        name = "curvature{}".format(self.k)
        ev = self.ev
        self.to_be_added[name] = np.nan_to_num(ev[:, 2] / (ev[:, 0] + ev[:, 1] + ev[:, 2]))


class Eigenentropy(EigenValuesScalarField):
    """
    """
    def compute(self):
        name = "eigenentropy{}".format(self.k)
        ev = self.ev
        result = np.zeros(ev.shape[0])
        for i in range(3):
            result += ev[:, i] * np.log(ev[:, i])
        self.to_be_added[name] = np.nan_to_num(-result)


class EigenSum(EigenValuesScalarField):
    """
    """
    def compute(self):
        name = "eigen_sum{}".format(self.k)
        self.to_be_added[name] = self.ev[:, 0] + self.ev[:, 1] + self.ev[:, 2]


class Linearity(EigenValuesScalarField):
    """
    """
    def compute(self):
        name = "linearity{}".format(self.k)
        ev = self.ev
        self.to_be_added[name] = np.nan_to_num((ev[:, 0] - ev[:, 1]) / ev[:, 0])


class Omnivariance(EigenValuesScalarField):
    """
    """
    def compute(self):
        name = "omnivariance{}".format(self.k)
        ev = self.ev
        self.to_be_added[name] = np.nan_to_num((ev[:, 0] * ev[:, 1] * ev[:, 2]) ** (1 / 3))


class Planarity(EigenValuesScalarField):
    """
    """
    def compute(self):
        name = "planarity{}".format(self.k)
        ev = self.ev
        self.to_be_added[name] = np.nan_to_num((ev[:, 1] - ev[:, 2]) / ev[:, 0])


class Sphericity(EigenValuesScalarField):
    """
    """
    def compute(self):
        name = "sphericity{}".format(self.k)
        ev = self.ev
        self.to_be_added[name] = np.nan_to_num(ev[:, 2] / ev[:, 0])
