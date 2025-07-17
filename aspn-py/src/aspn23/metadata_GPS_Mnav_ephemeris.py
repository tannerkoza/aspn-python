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
class MetadataGpsMnavEphemeris(AspnBase):
    """
    MNAV Ephemeris describing GPS satellite locations. Definitions and usage are covered in
    ICD-GPS-200L, Section 20.3.3.4 and following, with additional military use definitions and usage
    covered in ICD-GPS-700D.

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

    a_dot - float:
            Change rate of semi-major axis.

    delta_af_0 - float:
            SV clock bias correction provided by the Mnav Message Correction (MMC).

    delta_af_1 - float:
            SV clock drift correction provided by the Mnav Message Correction (MMC).

    delta_gamma - float:
            Ephemeris parameters correction provided by the Mnav Message Correction (MMC).

    delta_i - float:
            Angle of inclination correction provided by the Mnav Message Correction (MMC).

    delta_omega - float:
            Angle of Right Ascension correction provided by the Mnav Message Correction (MMC).

    delta_a - float:
            Semi-major axis correction provided by the Mnav Message Correction (MMC).

    isc_l1_m_e - float:
            L M1_E to L1 P(Y) inter-signal correction.

    isc_l2_m_e - float:
            L M2_E to L1 P(Y) inter-signal correction.

    isc_l1_m_s - float:
            L M1_S to L1 P(Y) inter-signal correction.

    isc_l2_m_s - float:
            L M2_S to L1 P(Y) inter-signal correction.

    isa_l2_py - float:
            L2 P(Y) to L1 P(Y) inter-signal accuracy index.

    isa_l1_m_e - float:
            L M1_E to L1 P(Y) inter-signal accuracy index.

    isa_l2_m_e - float:
            L M2_E to L1 P(Y) inter-signal accuracy index.

    isa_l1_m_s - float:
            L M1_S to L1 P(Y) inter-signal accuracy index.

    isa_l2_m_s - float:
            L M2_S to L1 P(Y) inter-signal accuracy index.
    """

    info: TypeMetadataheader
    time_of_validity: TypeTimestamp
    week_number: int
    prn: int
    clock: TypeSatnavClock
    orbit: TypeKeplerOrbit
    a_dot: float
    delta_af_0: float
    delta_af_1: float
    delta_gamma: float
    delta_i: float
    delta_omega: float
    delta_a: float
    isc_l1_m_e: float
    isc_l2_m_e: float
    isc_l1_m_s: float
    isc_l2_m_s: float
    isa_l2_py: float
    isa_l1_m_e: float
    isa_l2_m_e: float
    isa_l1_m_s: float
    isa_l2_m_s: float
