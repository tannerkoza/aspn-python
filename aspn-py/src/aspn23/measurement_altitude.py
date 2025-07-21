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


class MeasurementAltitudeReference(Enum):
    """
    Specifies altitude reference.
    """

    """
    height above ground level (AGL). Generally, position and terrain elevation data are required to
    convert between AGL and (MSL or HAE).
    """
    AGL = 0

    """
    height above mean sea level (MSL). Generally, position is required to convert between MSL and
    HAE.
    """
    MSL = 1

    """
    height above the WGS-84 ellipsoid (HAE), also known as geodetic altitude.
    """
    HAE = 2


class MeasurementAltitudeErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0

    """
    First-order Gauss-Markov (FOGM) process, defined by the following parameters
    (num_error_model_params = 3):
    1) standard_deviation, units: m, description: standard deviation of a scalar, first order
    Gauss-Markov measurement noise model.
    2) time_constant, units: seconds, description: time constant of a scalar, first order
    Gauss-Markov measurement noise model.
    3) initial_standard_deviation, units: m, description: initial standard deviation of a
    scalar, first order Gauss-Markov measurement noise model.
    """
    FOGM = 1


@dataclass
class MeasurementAltitude(AspnBase):
    """
    Height above a user-specified reference frame.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    reference - MeasurementAltitudeReference:
            Specifies altitude reference.

    altitude - float:
            Altitude in meters, with altitude defined by the enum, altitude_type.

    variance - float:
            Altitude variance.

    error_model - MeasurementAltitudeErrorModel:
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
    reference: MeasurementAltitudeReference
    altitude: float
    variance: float
    error_model: MeasurementAltitudeErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
