"""
This code is generated via https://git.aspn.us/pntos/firehose/-/blob/main/firehose/backends/aspn/aspn_yaml_to_python.py
DO NOT hand edit code.  Make any changes required using the firehose repo instead.
"""

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

import numpy as np

# Backwards compatibility for typing numpy arrays
NumpyArray = np.ndarray  # represents a 1 dimensional numpy array
NumpyMatrix = np.ndarray  # represents a 2 dimensional numpy array
try:
    from numpy.typing import NDArray

    NumpyArray = NumpyMatrix = NDArray
except ImportError:
    pass

from .type_integrity import TypeIntegrity


class TypeNavsimSatnavObsPseudorangeRateType(Enum):
    """
    Indicates the type of measurement contained in pseudorange_rate.
    """

    """
    Instantaneous Doppler at measurement time. Units = Hz Cov. Units = Hz^2
    """
    PSR_RATE_DOPPLER = 0

    """
    Change in pseudorange (integrated Doppler) over a specified time interval. The interval is
    provided in metadata_satnav_obs. Units = m Cov. Units = m^2
    """
    PSR_RATE_DELTA_RANGE = 1


class TypeNavsimSatnavObsIonoCorrectionSource(Enum):
    """
    Enumerated field which describes the source of the ionospheric delay correction.
    """

    """
    unknown source
    """
    UNKNOWN = 0

    """
    modeled using the ionospheric models in IS-GPS-200, IS-GPS-700, or IS-GPS-800
    """
    MODELED = 1

    """
    measured using dual frequencies
    """
    MEASURED = 2


@dataclass
class TypeNavsimSatnavObs:
    """
    Defines measurements that come from a satnav receiver tracking a single signal from a single
    satellite.

    ### Attributes

    satellite_system - str:
            String field which describes the Satellite System that generated the observations.

    signal_descriptor - str:
            Navsim signal

    prn - str:
            PRN code which identifies satellite (or slot number, in the case of GLONASS) for the
            observables.

    frequency - float:
            Center frequency of signal. For GLONASS, this should be the center frequency of the
            frequency-shifted signal with the k value taken into account.

    pseudorange - Optional[Optional[float]]:
            Pseudorange measurement

    pseudorange_variance - Optional[Optional[float]]:
            Variance of pseudorange measurement noise/multipath. This value must be provided if
            the pseudorange measurement is provided. Note: This is not intended to represent clock,
            atmospheric, or satellite position errors.

    pseudorange_rate_type - TypeNavsimSatnavObsPseudorangeRateType:
            Indicates the type of measurement contained in pseudorange_rate.

    pseudorange_rate - Optional[Optional[float]]:
            Measurement of the change in the pseudorange. See pseudorange_rate_type for details.

    pseudorange_rate_variance - Optional[Optional[float]]:
            Variance of pseudorange_rate noise. This value must be provided if the
            pseudorange_rate measurement is provided.

    carrier_phase - Optional[Optional[float]]:
            Carrier-phase (integrated Doppler) measurement. This measurement has an N cycle
            ambiguity (where N is unknown and arbitrary integer).

    carrier_phase_variance - Optional[Optional[float]]:
            Variance of carrier-phase noise/multipath. This value must be provided if the
            carrier-phase measurement is provided. Note: This is not intended to represent clock,
            atmospheric, or satellite position errors or the integer N ambiguity.

    c_n0 - Optional[Optional[float]]:
            Carrier to noise density ratio. C/N0 = 10[log10(S/N0)]

    lock_count - int:
            Number of observations with continuous tracking (no cycle slips). Value of -1 means
            that there is no loss of lock information available. First lock_count after cycle slip
            has occurred will have a value of 0. The lock_count only applies to the carrier-phase
            measurement.

    iono_correction_source - TypeNavsimSatnavObsIonoCorrectionSource:
            Enumerated field which describes the source of the ionospheric delay correction.

    iono_correction_applied - bool:
            Identifies if the ionospheric correction has been applied to the measurement. 1 =
            Applied.

    tropo_correction_applied - bool:
            Identifies if the tropospheric correction has been applied to the measurement. 1 =
            Applied.

    signal_bias_correction_applied - bool:
            Represents biases within the transmitting signal that are identified in the downlink
            data, Group Delay for LNAV, and Interchannel Signal Correction (ISC) message for MNAV.
            This term indicates whether the correction is known, being applied to Pseudorange
            Correction, and its associated estimated error applied to the ERE. When invalid the
            correction and estimated error are not known and therefore not applied. 1 = Valid.

    integrity - List[TypeIntegrity]:
            Measurement integrity. Includes the integrity method used and an integrity value
            (which is to be interpreted based upon the integrity method). The intent of allowing
            num_integrity > 1 is to report multiple integrity values based on multiple integrity
            methods.
    """

    satellite_system: str
    signal_descriptor: str
    prn: str
    frequency: float
    pseudorange: Optional[float]
    pseudorange_variance: Optional[float]
    pseudorange_rate_type: TypeNavsimSatnavObsPseudorangeRateType
    pseudorange_rate: Optional[float]
    pseudorange_rate_variance: Optional[float]
    carrier_phase: Optional[float]
    carrier_phase_variance: Optional[float]
    c_n0: Optional[float]
    lock_count: int
    iono_correction_source: TypeNavsimSatnavObsIonoCorrectionSource
    iono_correction_applied: bool
    tropo_correction_applied: bool
    signal_bias_correction_applied: bool
    integrity: List[TypeIntegrity]
