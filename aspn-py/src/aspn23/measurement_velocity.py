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


class MeasurementVelocityReferenceFrame(Enum):
    """
    Defines measurement reference.
    """

    """
    Velocity with respect to earth-centered inertial frame, expressed in an earth-centered inertial
    frame. See also ECi coordinate frame definition for x, y, and z definitions.
    """
    ECI = 0

    """
    Velocity with respect to earth-fixed frame expressed in earth-centered, earth-fixed (ECEF)
    frame. See also ECEF coordinate frame definition for x, y, and z definitions.
    """
    ECEF = 1

    """
    Local level, geodetic velocity: Velocity with respect to earth-fixed frame expressed in the
    North-East-Down (NED), local level frame. See also NED coordinate frame definition for N, E, and D
    definitions.
    """
    NED = 2

    """
    Velocity with respect to earth-fixed frame expressed in sensor coordinate frame. Sensor
    coordinate frame is defined in sensor metadataHeader.
    """
    SENSOR = 3


class MeasurementVelocityErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementVelocity(AspnBase):
    """
    Velocity is time rate of change of position with respect to the user-specified reference frame.
    May be one, two, or three dimensional.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    reference_frame - MeasurementVelocityReferenceFrame:
            Defines measurement reference.

    x - Optional[Optional[float]]:
            Velocity along the first axis of the measurement reference frame enumerated in
            reference_frame.

    y - Optional[Optional[float]]:
            Velocity along the second axis of the measurement reference frame enumerated in
            reference_frame.

    z - Optional[Optional[float]]:
            Velocity along the third axis of the measurement reference frame enumerated in
            reference_frame.

    covariance - NumpyMatrix[float]:
            Measurement error covariance or variance depending on measurement dimension.
            Dimensions of covariance must be num_meas²

    error_model - MeasurementVelocityErrorModel:
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
    reference_frame: MeasurementVelocityReferenceFrame
    x: Optional[float]
    y: Optional[float]
    z: Optional[float]
    covariance: NumpyMatrix[float]
    error_model: MeasurementVelocityErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
