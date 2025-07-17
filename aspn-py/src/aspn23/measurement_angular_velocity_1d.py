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


class MeasurementAngularVelocity1DSensorType(Enum):
    """
    sensor integration type
    """

    """
    Angular velocity integrated over last measurement period expressed as delta rotation in radians.
    Variance units are radians^2. Time of validity is the end of integration period. Integration period
    is from previous measurement to the current measurement.
    """
    INTEGRATED = 0

    """
    Angular velocity is sampled expressed in rad/s. Variance units are (rad/s)^2.
    """
    SAMPLED = 1


class MeasurementAngularVelocity1DErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementAngularVelocity1D(AspnBase):
    """
    Measures the angular velocity about the sense-axis. The sense-axis is the sensor frame x-axis as
    defined in mounting.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    sensor_type - MeasurementAngularVelocity1DSensorType:
            sensor integration type

    obs - float:
            Angular velocity defined in sensor_type enum.

    variance - float:
            Variance of the observation with units defined in sensor_type enum.

    error_model - MeasurementAngularVelocity1DErrorModel:
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
    sensor_type: MeasurementAngularVelocity1DSensorType
    obs: float
    variance: float
    error_model: MeasurementAngularVelocity1DErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
