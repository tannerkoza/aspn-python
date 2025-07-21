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


class MeasurementAccumulatedDistanceTraveledErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementAccumulatedDistanceTraveled(AspnBase):
    """
    Accumulated distance traveled over the time period from (time_of_validity - delta_t) to
    time_of_validity. For example, a sensor that travels (over the observation time period) in one
    direction 1m and reverses course to travel 1m back (and ends at its starting position) would have an
    accumulated distance traveled of 2m. Sensor examples may include an odometer or a wheel encoder.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    delta_t - float:
            Duration of the observation time interval. The time_of_validity denotes the end of
            the time interval. The start of the time interval is time_of_validity - delta_t.

    obs - float:
            Accumulated distance traveled over time period.

    variance - float:
            Accumulated distance traveled variance.

    error_model - MeasurementAccumulatedDistanceTraveledErrorModel:
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
    delta_t: float
    obs: float
    variance: float
    error_model: MeasurementAccumulatedDistanceTraveledErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
