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
from .type_remote_point import TypeRemotePoint
from .type_timestamp import TypeTimestamp


class MeasurementRangeToPointErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementRangeToPoint(AspnBase):
    """
    Range to a point is the distance between the sensor and the point at time_of_validity.
    Information about the remote point (its position and/or correspondence with previous or future
    appearances of this same point) may be determined using remote_point.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    remote_point - TypeRemotePoint:
            Position and/or correspondence information about the remote point.

    obs - float:
            Range to the remote point.

    variance - float:
            Variance of the range measurement.

    error_model - MeasurementRangeToPointErrorModel:
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
    remote_point: TypeRemotePoint
    obs: float
    variance: float
    error_model: MeasurementRangeToPointErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
