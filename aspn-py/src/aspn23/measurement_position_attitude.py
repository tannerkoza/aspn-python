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


class MeasurementPositionAttitudeReferenceFrame(Enum):
    """
    Defines measurement reference.
    """

    """
    Sensor position and attitude relative to the earth-centered, inertial (ECi) frame. Position is
    expressed where p1, p2, and p3 are the x, y, and z components of ECI position in meters,
    respectively. Attitude is expressed as a four-element quaternion. Error covariance is given using
    x-axis position error component (meters), y-axis position error component (meters), z-axis position
    error component (meters), tilt error about the x-axis (rad), tilt error about the y-axis (rad), and
    tilt error about the z-axis (rad). See also ECi coordinate frame definition for x, y, and z
    definitions.
    """
    ECI = 0

    """
    Sensor position, and attitude relative to the WGS-84 reference. Position is expressed where p1
    is the geodetic latitude in radians, p2 is the longitude in radians, and p3 is the geodetic altitude
    (or height above the WGS-84 ellipsoid) in meters. Attitude is the sensor frame rotation relative to
    the local level frame expressed as a four-element quaternion. Error covariance is given using
    north-south position error (meters), east-west position error (meters), geodetic altitude error
    (meters), tilt error about the north axis (rad), tilt error about the east axis (rad), and tilt
    error about the down axis (rad).
    """
    GEODETIC = 1


class MeasurementPositionAttitudeErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementPositionAttitude(AspnBase):
    """
    Position and attitude, also known as pose. Position is relative to a user-defined reference
    frame. Attitude expressed as a quaternion, a four element vector representation, the elements of
    which are functions of the orientation of the vector and the magnitude of the rotation.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    reference_frame - MeasurementPositionAttitudeReferenceFrame:
            Defines measurement reference.

    p1 - float:
            First position term as defined in enum.

    p2 - float:
            First position term as defined in enum.

    p3 - float:
            First position term as defined in enum.

    quaternion - NumpyArray[float]:
            Four element quaternion, q = [a, b, c, d], where a = cos(phi/2), b =
            (phi_x/phi)*sin(phi/2), c = (phi_y/phi)*sin(phi/2), and d = (phi_z/phi)*sin(phi/2). In
            this description, the vector [phi_x, phi_y, phi_z] represents the rotation vector that
            describes the frame rotation to be applied to the "reference" frame (ECI, ECEF, or NED)
            to rotate it into the axes that describe the measured attitude, and the value phi is the
            magnitude of the [phi_x, phi_y, phi_z] vector. See "conventions" documentation for more
            detailed information.

    covariance - NumpyMatrix[float]:
            Measurement error variance or covariance depending on measurement dimension.

    error_model - MeasurementPositionAttitudeErrorModel:
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
    reference_frame: MeasurementPositionAttitudeReferenceFrame
    p1: float
    p2: float
    p3: float
    quaternion: NumpyArray[float]
    covariance: NumpyMatrix[float]
    error_model: MeasurementPositionAttitudeErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
