import numpy as np
from .base import ScalarField


class RGBScalarField(ScalarField):
    def extract_info(self):
        self.rgb = self.pyntcloud.points[[
            "red", "green", "blue"]].values.astype("f")


class RGBIntensity(RGBScalarField):
    """ Red, green and blue intensity.
    """
    def compute(self):
        rgb_i = np.nan_to_num(
            self.rgb / np.sum(self.rgb, axis=1, keepdims=True))
        self.to_be_added["Ri"] = rgb_i[:, 0]
        self.to_be_added["Gi"] = rgb_i[:, 1]
        self.to_be_added["Bi"] = rgb_i[:, 2]


class RelativeLuminance(RGBScalarField):
    """ Similar to grayscale. Computed following Wikipedia.
    """
    def compute(self):
        self.rgb /= 255.
        coefficients = np.array([0.2125, 0.7154, 0.0721])
        self.to_be_added["relative_luminance"] = np.einsum(
            'ij, j', self.rgb, coefficients)


class HueSaturationValue(RGBScalarField):
    """ Hue, Saturation, Value colorspace.
    """
    def compute(self):
        rgb = self.rgb
        MAX = np.max(rgb, -1)
        MIN = np.min(rgb, -1)
        MAX_MIN = np.ptp(rgb, -1)

        H = np.empty_like(MAX)

        idx = rgb[:, 0] == MAX
        H[idx] = 60 * (rgb[idx, 1] - rgb[idx, 2]) / MAX_MIN[idx]
        H[np.logical_and(idx, rgb[:, 1] < rgb[:, 2])] += 360

        idx = rgb[:, 1] == MAX
        H[idx] = (60 * (rgb[idx, 2] - rgb[idx, 0]) / MAX_MIN[idx]) + 120

        idx = rgb[:, 2] == MAX
        H[idx] = (60 * (rgb[idx, 0] - rgb[idx, 1]) / MAX_MIN[idx]) + 240

        self.to_be_added["H"] = np.nan_to_num(H)
        self.to_be_added["S"] = np.nan_to_num(
            np.where(MAX == 0, 0, 1 - (MIN / MAX)))
        self.to_be_added["V"] = np.nan_to_num(MAX / 255 * 100)
