
"""
HAKUNA MATATA
"""

from .kdtree import (
    RadiusOutlierRemovalFilter,
    StatisticalOutlierRemovalFilter,

)
from .xyz import BoundingBoxFilter

ALL_FILTERS = {
    # XYZ
    "BBOX": BoundingBoxFilter,
    # KDTree
    "ROR": RadiusOutlierRemovalFilter,
    "SOR": StatisticalOutlierRemovalFilter
}
