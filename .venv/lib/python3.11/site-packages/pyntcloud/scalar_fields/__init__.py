from .eigenvalues import (
    Anisotropy,
    Curvature,
    Eigenentropy,
    EigenSum,
    Linearity,
    Omnivariance,
    Planarity,
    Sphericity
)
from .k_neighbors import (
    EigenDecomposition,
    EigenValues,
    UnorientedNormals,
)
from .normals import (
    InclinationDegrees,
    InclinationRadians,
    OrientationDegrees,
    OrientationRadians
)
from .rgb import (
    HueSaturationValue,
    RelativeLuminance,
    RGBIntensity
)
from .voxelgrid import (
    VoxelN,
    VoxelX,
    VoxelY,
    VoxelZ,
    EuclideanClusters
)
from .xyz import (
    PlaneFit,
    SphereFit,
    CustomFit,
    SphericalCoordinates,
    CylindricalCoordinates
)
ALL_SF = {
    # Eigenvalues
    'anisotropy': Anisotropy,
    'curvature': Curvature,
    'eigenentropy': Eigenentropy,
    'eigen_sum': EigenSum,
    'linearity': Linearity,
    'omnivariance': Omnivariance,
    'planarity': Planarity,
    'sphericity': Sphericity,
    # Kneighbors
    'eigen_decomposition': EigenDecomposition,
    'eigen_values': EigenValues,
    'normals': UnorientedNormals,
    # Normals
    'inclination_degrees': InclinationDegrees,
    'inclination_radians': InclinationRadians,
    'orientation_degrees': OrientationDegrees,
    'orientation_radians': OrientationRadians,
    # RGB
    'hsv': HueSaturationValue,
    'relative_luminance': RelativeLuminance,
    'rgb_intensity': RGBIntensity,
    # Voxelgrid
    'voxel_n': VoxelN,
    'voxel_x': VoxelX,
    'voxel_y': VoxelY,
    'voxel_z': VoxelZ,
    'euclidean_clusters': EuclideanClusters,
    # XYZ
    'custom_fit': CustomFit,
    'plane_fit': PlaneFit,
    'sphere_fit': SphereFit,
    'spherical_coords': SphericalCoordinates,
    'cylindrical_coords': CylindricalCoordinates
}
