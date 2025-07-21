"""
This code is generated via https://git.aspn.us/pntos/firehose/-/blob/main/firehose/backends/aspn/aspn_yaml_to_python.py
DO NOT hand edit code.  Make any changes required using the firehose repo instead.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

# Backwards compatibility for typing numpy arrays
NumpyArray = np.ndarray  # represents a 1 dimensional numpy array
NumpyMatrix = np.ndarray  # represents a 2 dimensional numpy array
try:
    from numpy.typing import NDArray

    NumpyArray = NumpyMatrix = NDArray
except ImportError:
    pass


class TypeSatnavSatelliteSystemSatelliteSystem(Enum):
    """
    Enumerated field which describes the Satellite System.
    """

    """
    GPS.
    """
    SYS_GPS = 0

    """
    Galileo.
    """
    SYS_GALILEO = 1

    """
    GLONASS.
    """
    SYS_GLONASS = 2

    """
    BeiDou.
    """
    SYS_BEIDOU = 3

    """
    SBAS.
    """
    SYS_SBAS = 4

    """
    QZSS.
    """
    SYS_QZSS = 5

    """
    IRNSS.
    """
    SYS_IRNSS = 6


@dataclass
class TypeSatnavSatelliteSystem:
    """
    Satellite system name

    ### Attributes

    satellite_system - TypeSatnavSatelliteSystemSatelliteSystem:
            Enumerated field which describes the Satellite System.
    """

    satellite_system: TypeSatnavSatelliteSystemSatelliteSystem
