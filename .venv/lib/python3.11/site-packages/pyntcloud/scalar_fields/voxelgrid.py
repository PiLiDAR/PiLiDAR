import numpy as np
from .base import ScalarField


class VoxelgridScalarField(ScalarField):

    def __init__(self, *, pyntcloud, voxelgrid_id):
        super().__init__(pyntcloud=pyntcloud)
        self.voxelgrid_id = voxelgrid_id

    def extract_info(self):
        self.voxelgrid = self.pyntcloud.structures[self.voxelgrid_id]

    def compute(self):
        pass


class VoxelX(VoxelgridScalarField):
    """Voxel index along x axis."""
    def compute(self):
        name = "{}({})".format("voxel_x", self.voxelgrid_id)
        self.to_be_added[name] = self.voxelgrid.voxel_x


class VoxelY(VoxelgridScalarField):
    """Voxel index along y axis."""
    def compute(self):
        name = "{}({})".format("voxel_y", self.voxelgrid_id)
        self.to_be_added[name] = self.voxelgrid.voxel_y


class VoxelZ(VoxelgridScalarField):
    """Voxel index along z axis."""
    def compute(self):
        name = "{}({})".format("voxel_z", self.voxelgrid_id)
        self.to_be_added[name] = self.voxelgrid.voxel_z


class VoxelN(VoxelgridScalarField):
    """Voxel index in 3D array using 'C' order."""
    def compute(self):
        name = "{}({})".format("voxel_n", self.voxelgrid_id)
        self.to_be_added[name] = self.voxelgrid.voxel_n


class EuclideanClusters(VoxelgridScalarField):
    """Assing corresponding cluster to each point inside each voxel."""
    def compute(self):
        name = "{}({})".format("clusters", self.voxelgrid_id)

        to_be_processed = np.zeros(self.voxelgrid.n_voxels, dtype=bool)
        to_be_processed[np.unique(self.voxelgrid.voxel_n)] = True

        clusters = np.zeros(self.voxelgrid.voxel_n.shape[0])

        C = 0
        while np.any(to_be_processed):
            Q = []
            Q.append(np.random.choice(np.where(to_be_processed)[0]))

            for voxel in Q:
                clusters[np.where(self.voxelgrid.voxel_n == voxel)[0]] = C
                to_be_processed[voxel] = False
                neighbors = self.voxelgrid.get_voxel_neighbors(voxel)
                for neighbor in neighbors:
                    if to_be_processed[neighbor]:
                        Q.append(neighbor)
                        to_be_processed[neighbor] = False
            C += 1

        self.to_be_added[name] = clusters
