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


class MeasurementSpecificForce1DSensorType(Enum):
    """
    sensor type
    """

    """
    Specific force integrated over last measurement period expressed as delta velocity in m/s.
    Variance units are (m/s)^2. Time of validity is the end of integration period. Integration period is
    from previous measurement to the current measurement.
    """
    INTEGRATED = 0

    """
    Specific force is sampled expressed in m/s/s. Variance units are (m/s/s)^2.
    """
    SAMPLED = 1


class MeasurementSpecificForce1DErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementSpecificForce1D(AspnBase):
    """
    Measures the apparent specific force along the sense-axis. Example sensors are an atomic
    interferometer, quantum gravimeter, or accelerometer. The apparent specific force is corrupted by
    inertial (Coriolis and Centrifugal) effects. The sense-axis is the sensor frame x-axis as defined in
    mounting.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    sensor_type - MeasurementSpecificForce1DSensorType:
            sensor type

    obs - float:
            Specific force defined in sensor_type enum.

    variance - float:
            Variance of the observation with units defined in sensor_type enum.

    error_model - MeasurementSpecificForce1DErrorModel:
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
    sensor_type: MeasurementSpecificForce1DSensorType
    obs: float
    variance: float
    error_model: MeasurementSpecificForce1DErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
