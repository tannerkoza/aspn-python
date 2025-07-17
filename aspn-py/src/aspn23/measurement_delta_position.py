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


class MeasurementDeltaPositionReferenceFrame(Enum):
    """
    Specifies measurement reference frame.
    """

    """
    Change in position along the N, E, and D axes of the locally level NED frame. Each direction is
    optional. term1, term2, and term3, if provided, describe the change in position along the N, E, and
    D axes, respectively. For example, a depth sensor may report a change in position of +2m along the D
    axis by omitting term1 and term2 (using the appropriate null) and using term3 = 2. This observation
    may be interpreted as (1) the sensor has no knowledge of the change in position in the N and E
    directions, and (2) the sensor observed that the position at the end of the observation time
    interval, time_of_validity, is 2 meters lower than the position at the start of the time interval,
    time_of_validity - delta_t. See also NED coordinate frame definition for N, E, and D definitions.
    """
    NED = 0

    """
    Change in position expressed in the sensor frame at the start of the measurement interval as
    defined in mounting. Each direction is optional. term1, term2, and term3, if provided, describe the
    change in position along the sensor x, y, and z axes, respectively.
    """
    SENSOR_START = 1

    """
    Change in position expressed in the sensor frame at the end of the measurement interval as
    defined in mounting. Each direction is optional. term1, term2, and term3, if provided, describe the
    change in position along the sensor x, y, and z axes, respectively.
    """
    SENSOR_END = 2


class MeasurementDeltaPositionErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementDeltaPosition(AspnBase):
    """
    Delta position is the change in position from the position at time (time_of_validity - delta_t)
    to the position at time (time_of_validity), where the change in position is reported along each axis
    of the reference frame. The reference frame is chosen by the user in the reference enum.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid at the end of the
            delta-position period.

    reference_frame - MeasurementDeltaPositionReferenceFrame:
            Specifies measurement reference frame.

    delta_t - float:
            Duration of the observation time interval. The time_of_validity denotes the end of
            the time interval. The start of the time interval is time_of_validity - delta_t.

    term1 - Optional[Optional[float]]:
            Delta position over the first dimension specified by the enum.

    term2 - Optional[Optional[float]]:
            Delta position over the second dimension specified by the enum.

    term3 - Optional[Optional[float]]:
            Delta position over the third dimension specified by the enum.

    covariance - NumpyMatrix[float]:
            Variance or covariance matrix depending on measurement dimension. Dimensions of
            covariance must be num_meas²

    error_model - MeasurementDeltaPositionErrorModel:
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
    reference_frame: MeasurementDeltaPositionReferenceFrame
    delta_t: float
    term1: Optional[float]
    term2: Optional[float]
    term3: Optional[float]
    covariance: NumpyMatrix[float]
    error_model: MeasurementDeltaPositionErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
