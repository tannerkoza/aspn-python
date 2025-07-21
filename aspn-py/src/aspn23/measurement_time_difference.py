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

from .aspn_base import AspnBase
from .type_header import TypeHeader
from .type_integrity import TypeIntegrity
from .type_timestamp import TypeTimestamp


class MeasurementTimeDifferenceErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementTimeDifference(AspnBase):
    """
    Time difference between two clocks or timing sources.

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

    time_diff_nsec - int:
            Whole number nanoseconds in time difference (time from clock_id1) - (time from
            clock_id2). Must be less than 1e9. This value shall be positive when (time from
            clock_id1) - (time from clock_id2) is greater than or equal to zero. This value shall be
            negative when (time from clock_id1) - (time from clock_id2) is less than
            zero.Additionally, the timestamp's digits of precision are included as
            `digits_of_precision`.

    time_diff_attosec - int:
            Whole number attoseconds in time difference (time from clock_id1) - (time from
            clock_id2). Must be less than 1e9. This value shall be positive when (time from
            clock_id1) - (time from clock_id2) is greater than or equal to zero. This value shall be
            negative when (time from clock_id1) - (time from clock_id2) is less than zero. This is
            only needed if the time of validity is known to more precision than 1ns; otherwise, set
            time_of_validity_attosec = 0. Be sure to report the timestamp's digits of precision. The
            time difference in seconds is calculated as time difference = time_diff_nsec * 1e-9 +
            time_diff_attosec * 1e-18 It shall be assumed that time_diff_nsec and time_diff_attosec
            have the same sign. Therefore, the time difference is positive when (time from
            clock_id1) - (time from clock_id2) is positive and the time difference is negative when
            (time from clock_id1) - (time from clock_id2) is negative. A cautionary note: Both
            timediff_nsec and time_diff_attosec are signed integers and may be positive or negative.
            It is mandated to make timediff_nsec and time_diff_attosec the same sign; however, care
            should be taken when constructing the observation components to be sure the summation
            produces the desired result.

    digits_of_precision - int:
            Defines how many decimal digits of precision are represented in the observation
            after the decimal point. The precision of the observation is 10E-N, where N is
            digits_of_precision. A value of 3 would mean that the observation has precision at the
            10E-3 (millisecond) level, a value of 6 would mean the observation has precision at the
            10E-6 (microsecond) level, etc.

    variance - float:
            Variance of the error in the time difference measurement.

    error_model - MeasurementTimeDifferenceErrorModel:
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
    time_diff_nsec: int
    time_diff_attosec: int
    digits_of_precision: int
    variance: float
    error_model: MeasurementTimeDifferenceErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
