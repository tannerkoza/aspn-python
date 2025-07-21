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


class MeasurementSpeedReference(Enum):
    """
    Defines measurement reference.
    """

    """
    Speed with respect to air measured as true airspeed (TAS) with respect to air mass expressed in
    sensor coordinate frame.
    """
    TAS = 0

    """
    Speed with respect to air measured as indicated airspeed (IAS) with respect to air mass
    expressed in sensor coordinate frame; for example, dynamic pressure from a pitot tube converted to
    airspeed without compensation for instrument error, position, altitude, or temperature.
    """
    IAS = 1

    """
    Speed with respect to water. Speed with respect to water expressed in sensor coordinate frame.
    """
    WATER = 2

    """
    Speed with respect to ground. An example source is a ground Doppler radar.
    """
    GROUND = 3


class MeasurementSpeedErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementSpeed(AspnBase):
    """
    Speed is the magnitude of the velocity vector with respect to the user-specified reference
    frame.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    reference - MeasurementSpeedReference:
            Defines measurement reference.

    speed - float:
            Speed as the magnitude of the velocity.

    variance - float:
            Speed variance.

    error_model - MeasurementSpeedErrorModel:
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
    reference: MeasurementSpeedReference
    speed: float
    variance: float
    error_model: MeasurementSpeedErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
