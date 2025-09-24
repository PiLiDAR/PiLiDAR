from abc import ABC, abstractmethod
from collections import OrderedDict


class ScalarField(ABC):
    """Base class for scalar fields."""

    def __init__(self, *, pyntcloud):
        self.pyntcloud = pyntcloud
        self.to_be_added = OrderedDict()

    def get_and_set(self):
        sf_added = []
        for k, v in self.to_be_added.items():
            sf_added.append(k)
            self.pyntcloud.points[k] = v

        if len(sf_added) == 1:
            return sf_added[0]
        else:
            return sf_added

    @abstractmethod
    def extract_info(self):
        pass

    @abstractmethod
    def compute(self):
        pass
