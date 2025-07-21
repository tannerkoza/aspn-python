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


class MeasurementPositionReferenceFrame(Enum):
    """
    Defines measurement reference.
    """

    """
    Sensor position relative to the earth-centered, inertial (ECi) frame expressed where term1,
    term2, and term3 are the x, y, and z components of ECI position in meters, respectively. Each term
    is optional. Error covariance is given using x-axis position error component (meters), y-axis
    position error component (meters), and z-axis position error component (meters). In the case of
    nulled position components, the corresponding error covariance terms are omitted. See also ECi
    coordinate frame definition for x, y, and z definitions.
    """
    ECI = 0

    """
    Sensor position relative to the WGS-84 reference expressed using term1 as geodetic latitude in
    radians, term2 as longitude in radians, and term3 as geodetic altitude (or height above the WGS-84
    ellipsoid) in meters. Each term is optional. Error covariance is given using north-south position
    error in meters (an expression of geodetic latitude error), east-west position error in meters (an
    expression of longitude error in meters), and geodetic altitude error in meters. In the case of
    nulled position components, the corresponding error covariance terms are omitted.
    """
    GEODETIC = 1


class MeasurementPositionErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementPosition(AspnBase):
    """
    One, two, or three dimensional position relative to a user-defined reference frame.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    reference_frame - MeasurementPositionReferenceFrame:
            Defines measurement reference.

    term1 - Optional[Optional[float]]:
            First position term as defined in enum.

    term2 - Optional[Optional[float]]:
            Second position term as defined in enum.

    term3 - Optional[Optional[float]]:
            Third position term as defined in enum.

    covariance - NumpyMatrix[float]:
            Error covariance or variance depending on observation dimension. Dimensions of
            covariance must be num_meas²

    error_model - MeasurementPositionErrorModel:
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
    reference_frame: MeasurementPositionReferenceFrame
    term1: Optional[float]
    term2: Optional[float]
    term3: Optional[float]
    covariance: NumpyMatrix[float]
    error_model: MeasurementPositionErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
