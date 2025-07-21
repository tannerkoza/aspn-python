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


class TypeSatnavTimeTimeReference(Enum):
    """
    Reference time system used to express the data in this message. In a multi-GNSS receiver
    (GPS/GLONASS/Galileo/QZSS/BeiDou) all pseudorange observations must refer to one receiver clock
    only. The receiver clock time of the measurement is the receiver clock time of the received signals.
    It is identical for the phase and range measurements and is identical for all satellites observed in
    a given epoch.
    """

    """
    GPS system time.
    """
    TIME_GPS = 0

    """
    Galileo system time.
    """
    TIME_GALILEO = 1

    """
    BeiDou system time.
    """
    TIME_BEIDOU = 2

    """
    GLONASS system time.
    """
    TIME_GLONASS = 3


@dataclass
class TypeSatnavTime:
    """
    Satellite system time

    ### Attributes

    week_number - int:
            Full Week number since zero epoch at which the data provided in this message is
            valid, expressed in time system defined by time_reference enum (below).

    seconds_of_week - float:
            Seconds since start of current week at which the data provided in this message is
            valid, expressed in time system defined by time_reference enum (below).

    time_reference - TypeSatnavTimeTimeReference:
            Reference time system used to express the data in this message. In a multi-GNSS
            receiver (GPS/GLONASS/Galileo/QZSS/BeiDou) all pseudorange observations must refer to
            one receiver clock only. The receiver clock time of the measurement is the receiver
            clock time of the received signals. It is identical for the phase and range measurements
            and is identical for all satellites observed in a given epoch.
    """

    week_number: int
    seconds_of_week: float
    time_reference: TypeSatnavTimeTimeReference
