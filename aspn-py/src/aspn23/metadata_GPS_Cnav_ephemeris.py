from dataclasses import dataclass
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


@dataclass
class MetadataGpsCnavEphemeris(AspnBase):
    """
    CNAV Ephemeris describing GPS satellite locations. Definitions and usage are covered in
    ICD-GPS-200L, Section 20.3.3.4 and following.

    ### Attributes

    info - TypeMetadataheader:
            Standard ASPN metadata header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    week_number - int:
            Full GPS week number calculated from the Modulo 1024 WN in Subframe 1 and the number
            of GPS week rollovers

    prn - int:
            Satellite PRN number.

    clock - TypeSatnavClock:
            GNSS broadcast parameters required to calculate sv clock corrections.

    orbit - TypeKeplerOrbit:
            Keplerian orbit parameters required to calculate satellite position.

    t_gd - float:
            Group delay differential between L1 and L2.

    iodc - int:
            Issue of Data Clock. 10 bit value from Subframe 1

    iode - int:
            Issue of Data Ephemeris. 8 bits repeated in Subframe 2 and Subframe 3. Should match
            the 8 LSBs of the IODC.

    isc_l1_ca - float:
            L1 P(Y) to L1 C/A inter-signal correction.

    isc_l2_c - float:
            L1 P(Y) to L2C inter-signal correction.

    isc_l5_i5 - float:
            L1 P(Y) to L5 I5 inter-signal correction.

    isc_l5_q5 - float:
            L1 P(Y) to L5 Q5 inter-signal correction.

    delta_a_0 - float:
            Semi-major axis difference at reference time.

    a_dot - float:
            Change rate of semi-major axis.
    """

    info: TypeMetadataheader
    time_of_validity: TypeTimestamp
    week_number: int
    prn: int
    clock: TypeSatnavClock
    orbit: TypeKeplerOrbit
    t_gd: float
    iodc: int
    iode: int
    isc_l1_ca: float
    isc_l2_c: float
    isc_l5_i5: float
    isc_l5_q5: float
    delta_a_0: float
    a_dot: float
