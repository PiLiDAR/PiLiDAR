"""Compatibility adapter that proxies to the upstream-style :class:`Lidar` driver.

The original PiLiDAR project recorded LiDAR data directly via the serial
protocol using :mod:`lib.lidar_driver`.  Earlier revisions of this fork wrapped

the vendor SDK instead, which produced diverging raw data.  To keep existing
imports working while reverting to the proven pipeline we expose the same
interface but delegate all behaviour to :class:`lib.lidar_driver.Lidar`.
"""

from __future__ import annotations

from lib.lidar_driver import Lidar


class STL27LAdapter(Lidar):
    """Thin wrapper around :class:`Lidar` for backwards compatibility."""

    def __init__(self, config, port: str | None = None, baud: int | None = None):
        # When a custom port or baudrate is provided we update the configuration
        # so the base driver picks it up during initialisation.
        device = config.get("LIDAR", "DEVICE")
        if port is not None:
            config.set(port, "LIDAR", device, "PORT")
            config.PORT = port
        if baud is not None:
            config.set(baud, "LIDAR", device, "BAUDRATE")
        super().__init__(config)
