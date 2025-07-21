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


class MeasurementDirectionOfMotion2DReference(Enum):
    """
    Specifies measurement reference.
    """

    """
    Direction of motion expressed in the two-dimensional North-East plane of the locally level NED
    frame as the angle between the N-axis and the direction of motion. Positive angle is defined as a
    rotation towards the E-axis. See also NED coordinate frame definition for N, E, and D definitions.
    """
    NE = 0

    """
    Direction of motion expressed in the two-dimensional North-Down plane of the locally level NED
    frame as the angle between the N-axis and the direction of motion. Positive angle is defined as a
    rotation towards the D-axis. See also NED coordinate frame definition for N, E, and D definitions.
    """
    ND = 1

    """
    Direction of motion expressed in the two-dimensional East-Down plane of the locally level NED
    frame as the angle between the E-axis and the direction of motion. Positive angle is defined as a
    rotation towards the D-axis. See also NED coordinate frame definition for N, E, and D definitions.
    """
    ED = 2

    """
    Direction of motion expressed in the sensor frame as the angle between the sensor frame x-axis
    and the direction of motion. Positive angle is defined as a rotation towards the sensor frame
    y-axis. (Sensor frame is two dimensional, x and y, as defined in mounting.)
    """
    SENSOR = 3


class MeasurementDirectionOfMotion2DErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementDirectionOfMotion2D(AspnBase):
    """
    Direction of motion represented in a two-dimensional reference frame as defined by the enum.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    reference - MeasurementDirectionOfMotion2DReference:
            Specifies measurement reference.

    obs - float:
            Observations as specified in the reference enum.

    variance - float:
            Measurement error variance.

    error_model - MeasurementDirectionOfMotion2DErrorModel:
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
    reference: MeasurementDirectionOfMotion2DReference
    obs: float
    variance: float
    error_model: MeasurementDirectionOfMotion2DErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
