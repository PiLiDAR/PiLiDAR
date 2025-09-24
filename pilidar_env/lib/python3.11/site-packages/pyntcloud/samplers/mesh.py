import numpy as np
import pandas as pd

from .base import Sampler
from ..geometry.areas import triangle_area_multi


class MeshSampler(Sampler):
    """
    """

    def __init__(self, *, pyntcloud, rgb=False, normals=False):
        super().__init__(pyntcloud=pyntcloud)
        self.rgb = rgb
        self.normals = normals

    def extract_info(self):
        v1, v2, v3 = self.pyntcloud.get_mesh_vertices(
            rgb=self.rgb, normals=self.normals)

        self.v1_xyz = v1[:, :3]
        self.v2_xyz = v2[:, :3]
        self.v3_xyz = v3[:, :3]

        if self.rgb:
            self.v1_rgb = v1[:, 3:6]
            self.v2_rgb = v2[:, 3:6]
            self.v3_rgb = v3[:, 3:6]

            if self.normals:
                self.v1_normals = v1[:, 6:]
                self.v2_normals = v2[:, 6:]
                self.v3_normals = v3[:, 6:]

        elif self.normals:
            self.v1_normals = v1[:, 3:6]
            self.v2_normals = v2[:, 3:6]
            self.v3_normals = v3[:, 3:6]


class RandomMeshSampler(MeshSampler):
    """ Sample points adjusting probabilities according to triangle area.

    Parameters
    ----------
    n: int
        Number of points to be sampled.

    rgb: bool, optional
        Default: False
        Indicates if RGB values will also be sampled.

    normals: bool, optional
        Default: False
        Indicates if normals will also be sampled.

    """

    def __init__(self, *, pyntcloud, n, rgb=False, normals=False):
        super().__init__(pyntcloud=pyntcloud, rgb=rgb, normals=normals)
        self.n = n

    def compute(self):
        areas = triangle_area_multi(self.v1_xyz, self.v2_xyz, self.v3_xyz)
        probabilities = areas / np.sum(areas)
        random_idx = np.random.choice(
            np.arange(len(areas)), size=self.n, p=probabilities)

        v1_xyz = self.v1_xyz[random_idx]
        v2_xyz = self.v2_xyz[random_idx]
        v3_xyz = self.v3_xyz[random_idx]

        # (n, 1) the 1 is for broadcasting
        u = np.random.uniform(low=0., high=1., size=(self.n, 1))
        v = np.random.uniform(low=0., high=1-u, size=(self.n, 1))

        result = pd.DataFrame()

        result_xyz = (v1_xyz * u) + (v2_xyz * v) + ((1 - (u + v)) * v3_xyz)
        result_xyz = result_xyz.astype(np.float32)

        result["x"] = result_xyz[:, 0]
        result["y"] = result_xyz[:, 1]
        result["z"] = result_xyz[:, 2]

        if self.rgb:
            v1_rgb = self.v1_rgb[random_idx]
            v2_rgb = self.v2_rgb[random_idx]
            v3_rgb = self.v3_rgb[random_idx]

            result_rgb = (v1_rgb * u) + (v2_rgb * v) + ((1 - (u + v)) * v3_rgb)
            result_rgb = result_rgb.astype(np.uint8)

            result["red"] = result_rgb[:, 0]
            result["green"] = result_rgb[:, 1]
            result["blue"] = result_rgb[:, 2]

        if self.normals:
            v1_normals = self.v1_normals[random_idx]
            v2_normals = self.v2_normals[random_idx]
            v3_normals = self.v3_normals[random_idx]

            sum_normals = v1_normals + v2_normals + v3_normals

            result_normals = sum_normals / np.linalg.norm(sum_normals, axis=1)[..., None]
            result_normals = result_normals.astype(np.float32)

            result["nx"] = result_normals[:, 0]
            result["ny"] = result_normals[:, 1]
            result["nz"] = result_normals[:, 2]

        return result
