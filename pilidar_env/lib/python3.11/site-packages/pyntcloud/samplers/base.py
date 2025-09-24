from abc import ABC, abstractmethod


class Sampler(ABC):
    """Base class for sampling methods."""

    def __init__(self, *, pyntcloud):
        self.pyntcloud = pyntcloud

    @abstractmethod
    def extract_info(self):
        pass

    @abstractmethod
    def compute(self):
        pass
