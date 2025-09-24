import numpy as np

from scipy.spatial import KDTree

from .base import Structure
from ..plot.voxelgrid import plot_voxelgrid
from ..utils.array import cartesian

try:
    from ..utils.numba import groupby_max, groupby_count, groupby_sum
    is_numba_avaliable = True
except ImportError:
    is_numba_avaliable = False


class VoxelGrid(Structure):

    def __init__(self,
                 *,
                 points,
                 colors=None,
                 n_x=1, n_y=1, n_z=1,
                 size_x=None, size_y=None, size_z=None,
                 regular_bounding_box=True):
        """Grid of voxels with support for different build methods.

        Parameters
        ----------
        points: (N, 3) numpy.array
        colors: (N, 3) numpy.array, optional
            Default None.
            If not None, color for each voxel will be computed.
        n_x, n_y, n_z :  int, optional
            Default: 1
            The number of segments in which each axis will be divided.
            Ignored if corresponding size_x, size_y or size_z is not None.
        size_x, size_y, size_z : float, optional
            Default: None
            The desired voxel size along each axis.
            If not None, the corresponding n_x, n_y or n_z will be ignored.
        regular_bounding_box : bool, optional
            Default: True
            If True, the bounding box of the point cloud will be adjusted
            in order to have all the dimensions of equal length.
        """
        super().__init__(points=points)
        self.colors = colors
        self.x_y_z = np.asarray([n_x, n_y, n_z])
        self.sizes = np.asarray([size_x, size_y, size_z])
        self.regular_bounding_box = regular_bounding_box

        self.id = None
        self.xyzmin, self.xyzmax = None, None
        self.segments = None
        self.shape = None
        self.n_voxels = None
        self.voxel_x, self.voxel_y, self.voxel_z = None, None, None
        self.voxel_n = None
        self.voxel_centers = None
        self.voxel_colors = None

    @classmethod
    def extract_info(cls, pyntcloud):
        """ABC API"""
        try:
            colors = pyntcloud.points[['red', 'green', 'blue']].to_numpy()
        except KeyError:
            colors = None

        info = {
            "points": pyntcloud.xyz,
            "colors": colors
        }
        return info

    def compute(self):
        """ABC API."""
        xyzmin = self._points.min(0)
        xyzmax = self._points.max(0)
        xyz_range = self._points.ptp(0)

        if self.regular_bounding_box:
            #: adjust to obtain a minimum bounding box with all sides of equal length
            margin = max(xyz_range) - xyz_range
            xyzmin = xyzmin - margin / 2
            xyzmax = xyzmax + margin / 2

        for n, size in enumerate(self.sizes):
            if size is None:
                continue
            margin = (((self._points.ptp(0)[n] // size) + 1) * size) - self._points.ptp(0)[n]
            xyzmin[n] -= margin / 2
            xyzmax[n] += margin / 2
            self.x_y_z[n] = ((xyzmax[n] - xyzmin[n]) / size).astype(int)

        self.xyzmin = xyzmin
        self.xyzmax = xyzmax

        segments = []
        shape = []
        for i in range(3):
            # note the +1 in num
            s, step = np.linspace(xyzmin[i],
                                  xyzmax[i],
                                  num=(self.x_y_z[i] + 1),
                                  retstep=True)
            segments.append(s)
            shape.append(step)

        self.segments = segments
        self.shape = shape

        self.n_voxels = np.prod(self.x_y_z)

        self.id = "V({},{},{})".format(self.x_y_z, self.sizes, self.regular_bounding_box)

        # find where each point lies in corresponding segmented axis
        # -1 so index are 0-based; clip for edge cases
        self.voxel_x = np.clip(np.searchsorted(self.segments[0], self._points[:, 0]) - 1, 0, self.x_y_z[0])
        self.voxel_y = np.clip(np.searchsorted(self.segments[1], self._points[:, 1]) - 1, 0, self.x_y_z[1])
        self.voxel_z = np.clip(np.searchsorted(self.segments[2], self._points[:, 2]) - 1, 0,  self.x_y_z[2])
        self.voxel_n = np.ravel_multi_index([self.voxel_x, self.voxel_y, self.voxel_z], self.x_y_z)

        # compute center of each voxel
        midsegments = [(self.segments[i][1:] + self.segments[i][:-1]) / 2 for i in range(3)]
        self.voxel_centers = cartesian(midsegments).astype(np.float32)

        # compute voxel colors
        if self.colors is not None:
            order = np.argsort(self.voxel_n)
            _, breaks, counts = np.unique(self.voxel_n[order],
                                          return_index=True,
                                          return_counts=True)
            repeated_counts = np.repeat(counts[:, None], 3, axis=1)
            # why square? watch this: https://www.youtube.com/watch?v=LKnqECcg6Gw
            squared_colors = np.square(self.colors[order].astype(np.int64))
            summed_colors = np.add.reduceat(squared_colors,
                                            breaks,
                                            axis=0)
            averaged_colors = np.sqrt(summed_colors / repeated_counts)
            self.voxel_colors = np.rint(averaged_colors).astype(np.uint8)

    def query(self, points):
        """ABC API. Query structure.

        TODO Make query_voxelgrid an independent function, and add a light
        save mode where only segments and x_y_z are saved.
        """
        voxel_x = np.clip(np.searchsorted(
            self.segments[0], points[:, 0]) - 1, 0, self.x_y_z[0])
        voxel_y = np.clip(np.searchsorted(
            self.segments[1], points[:, 1]) - 1, 0, self.x_y_z[1])
        voxel_z = np.clip(np.searchsorted(
            self.segments[2], points[:, 2]) - 1, 0, self.x_y_z[2])
        voxel_n = np.ravel_multi_index([voxel_x, voxel_y, voxel_z], self.x_y_z)

        return voxel_n

    def get_feature_vector(self, mode="binary"):
        """Return a vector of size self.n_voxels. See mode options below.

        Parameters
        ----------
        mode: str in available modes. See Notes
            Default "binary"

        Returns
        -------
        feature_vector: [n_x, n_y, n_z] ndarray
            See Notes.

        Notes
        -----
        Available modes are:

        binary
            0 for empty voxels, 1 for occupied.

        density
            number of points inside voxel / total number of points.

        TDF
            Truncated Distance Function. Value between 0 and 1 indicating the distance
            between the voxel's center and the closest point. 1 on the surface,
            0 on voxels further than 2 * voxel side.

        x_max, y_max, z_max
            Maximum coordinate value of points inside each voxel.

        x_mean, y_mean, z_mean
            Mean coordinate value of points inside each voxel.
        """
        vector = np.zeros(self.n_voxels)

        if mode == "binary":
            vector[np.unique(self.voxel_n)] = 1

        elif mode == "density":
            count = np.bincount(self.voxel_n)
            vector[:len(count)] = count
            vector /= len(self.voxel_n)

        elif mode == "TDF":
            # truncation = np.linalg.norm(self.shape)
            kdt = KDTree(self._points)
            vector, i = kdt.query(self.voxel_centers, workers=-1)

        elif mode.endswith("_max"):
            if not is_numba_avaliable:
                raise ImportError("numba is required to compute {}".format(mode))
            axis = {"x_max": 0, "y_max": 1, "z_max": 2}
            vector = groupby_max(self._points, self.voxel_n, axis[mode], vector)

        elif mode.endswith("_mean"):
            if not is_numba_avaliable:
                raise ImportError("numba is required to compute {}".format(mode))
            axis = {"x_mean": 0, "y_mean": 1, "z_mean": 2}
            voxel_sum = groupby_sum(self._points, self.voxel_n, axis[mode], np.zeros(self.n_voxels))
            voxel_count = groupby_count(self._points, self.voxel_n, np.zeros(self.n_voxels))
            vector = np.nan_to_num(voxel_sum / voxel_count)

        else:
            raise NotImplementedError("{} is not a supported feature vector mode".format(mode))

        return vector.reshape(self.x_y_z)

    def get_voxel_neighbors(self, voxel):
        """Get valid, non-empty 26 neighbors of voxel.

        Parameters
        ----------
        voxel: int in self.set_voxel_n

        Returns
        -------
        neighbors: list of int
            Indices of the valid, non-empty 26 neighborhood around voxel.
        """

        x, y, z = np.unravel_index(voxel, self.x_y_z)

        valid_x = []
        valid_y = []
        valid_z = []
        if x - 1 >= 0:
            valid_x.append(x - 1)
        if y - 1 >= 0:
            valid_y.append(y - 1)
        if z - 1 >= 0:
            valid_z.append(z - 1)

        valid_x.append(x)
        valid_y.append(y)
        valid_z.append(z)

        if x + 1 < self.x_y_z[0]:
            valid_x.append(x + 1)
        if y + 1 < self.x_y_z[1]:
            valid_y.append(y + 1)
        if z + 1 < self.x_y_z[2]:
            valid_z.append(z + 1)

        valid_neighbor_indices = cartesian((valid_x, valid_y, valid_z))

        ravel_indices = np.ravel_multi_index((valid_neighbor_indices[:, 0],
                                              valid_neighbor_indices[:, 1],
                                              valid_neighbor_indices[:, 2]), self.x_y_z)

        return [x for x in ravel_indices if x in np.unique(self.voxel_n)]

    def plot(self,
             d=3,
             mode="binary",
             backend='pythreejs',
             cmap="Oranges",
             axis=False,
             output_name=None,
             width=800,
             height=500):

        return plot_voxelgrid(self,
                              d=d,
                              mode=mode,
                              backend=backend,
                              cmap=cmap,
                              axis=axis,
                              output_name=output_name,
                              width=width,
                              height=height)
