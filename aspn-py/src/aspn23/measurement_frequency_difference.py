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

from .aspn_base import AspnBase
from .type_header import TypeHeader
from .type_integrity import TypeIntegrity
from .type_timestamp import TypeTimestamp


class MeasurementFrequencyDifferenceErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementFrequencyDifference(AspnBase):
    """
    Frequency difference between two clocks or timing sources.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    time_of_validity_attosec - int:
            Whole number of attoseconds used to provide additional timestamp fidelity describing
            the time at which the measurement is considered to be valid. This is only needed if the
            time of validity is known to more precision than 1ns; otherwise, set
            time_of_validity_attosec = 0. Be sure to report the timestamp's digits of precision in
            the `type_metadataheader` as part of the metadata. The full timestamp in nanoseconds is
            calculated as time_of_validity + ( time_of_validity_attosec * 1E-9 ) and if needed, a
            timestamp in seconds is calculated as time_of_validity * 1E-9 + time_of_validity_attosec
            * 1E-18 (Recall time_of_validity is in nanoseconds and time_of_validity_attosec is in
            attoseconds.) Furthermore, both time_of_validity and time_of_validity_attosec are signed
            integers and may be positive or negative. It is recommended to make time_of_validity and
            time_of_validity_attosec the same sign. Care should be taken when constructing the
            timestamp components to be sure the summation produces the desired result.

    clock_id1 - int:
            Identifier for clock/timing source 1. See clock_identifiers.md for a full
            description, but the summary is as follows: 0 = ASPN System Time 1 = International
            Atomic Time (TAI) 2 = Universal Coordinated Time (UTC) 3 = GPS System Time 4 = Galileo
            System Time 5 = GLONASS System Time 6 = BeiDou System Time 7-50: Reserved for future
            additional time scale representations.

    clock_id2 - int:
            Identifier for clock/timing source 2. See clock_identifiers.md for a full
            description, but the summary is as follows: 0 = ASPN System Time 1 = International
            Atomic Time (TAI) 2 = Universal Coordinated Time (UTC) 3 = GPS System Time 4 = Galileo
            System Time 5 = GLONASS System Time 6 = BeiDou System Time 7-50: Reserved for future
            additional time scale representations.

    freq_diff - float:
            Frequency difference formed as (frequency from clock_id1) - (frequency from
            clock_id2).

    variance - float:
            Variance of the error in the frequency difference measurement.

    error_model - MeasurementFrequencyDifferenceErrorModel:
            Defines an optional error model for other than zero-mean, additive, white Gaussian
            noise (AWGN).

    error_model_params - NumpyArray[float]:
            Error model parameters that characterize the optional error model.

    integrity - List[TypeIntegrity]:
            Measurement integrity. Includes the integrity method used and an integrity value
            (which is to be interpreted based upon the integrity method). The intent of allowing
            num_integrity > 1 is to report multiple integrity values based on multiple integrity
            methods.
    """

    header: TypeHeader
    time_of_validity: TypeTimestamp
    time_of_validity_attosec: int
    clock_id1: int
    clock_id2: int
    freq_diff: float
    variance: float
    error_model: MeasurementFrequencyDifferenceErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
