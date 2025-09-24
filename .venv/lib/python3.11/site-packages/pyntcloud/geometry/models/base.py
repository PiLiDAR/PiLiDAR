from abc import ABC, abstractmethod


class GeometryModel(ABC):

    @abstractmethod
    def from_k_points(self, points):
        pass

    @abstractmethod
    def get_projections(self, points, only_distances=False):
        pass
