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

from .aspn_base import AspnBase
from .type_kepler_orbit import TypeKeplerOrbit
from .type_metadataheader import TypeMetadataheader
from .type_satnav_clock import TypeSatnavClock
from .type_timestamp import TypeTimestamp


class MetadataGalileoEphemerisNavMsgType(Enum):
    """
    Enumerated field which describes the GALILEO message type used to define clock correction
    parameters.
    """

    """
    I/NAV ephemeris.
    """
    INAV = 0

    """
    F/NAV ephemeris.
    """
    FNAV = 1


@dataclass
class MetadataGalileoEphemeris(AspnBase):
    """
    GALILEO Ephemeris describing satellite locations. Definitions and usage are covered in OS SIS
    ICD, Issue 2.0, January 2021, Section 4.2, 4.3, and 5.1.

    ### Attributes

    info - TypeMetadataheader:
            Standard ASPN metadata header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    nav_msg_type - MetadataGalileoEphemerisNavMsgType:
            Enumerated field which describes the GALILEO message type used to define clock
            correction parameters.

    prn - int:
            Satellite PRN number.

    clock - TypeSatnavClock:
            GNSS broadcast parameters required to calculate sv clock corrections.

    orbit - TypeKeplerOrbit:
            Keplerian orbit parameters required to calculate satellite position.

    bgd - float:
            Broadcast group delay (E1-E5b if F/Nav, E1-E5b if I/NAV)
    """

    info: TypeMetadataheader
    time_of_validity: TypeTimestamp
    nav_msg_type: MetadataGalileoEphemerisNavMsgType
    prn: int
    clock: TypeSatnavClock
    orbit: TypeKeplerOrbit
    bgd: float
