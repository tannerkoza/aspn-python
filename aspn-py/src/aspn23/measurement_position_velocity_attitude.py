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


class MeasurementPositionVelocityAttitudeReferenceFrame(Enum):
    """
    Defines measurement reference.
    """

    """
    Sensor position, velocity, and attitude relative to the earth-centered, inertial (ECi) frame.
    Position is expressed where p1, p2, and p3 are the x, y, and z components of ECI position in meters,
    respectively. Velocity is expressed where v1, v2, and v3 are the x, y, and z components of velocity
    with respect to the ECi frame in m/s, respectively. Attitude is expressed as a four-element
    quaternion. Position, in whole or in part, is optional. Velocity, in whole or in part, is optional.
    Attitude is optional, but if included, must be 3 dimensional. Error covariance is given using x-axis
    position error component (meters), y-axis position error component (meters), z-axis position error
    component (meters), x-axis velocity error component (m/s), y-axis velocity error component (m/s),
    z-axis velocity error component (m/s), tilt error about the x-axis (rad), tilt error about the
    y-axis (rad), and tilt error about the z-axis (rad). In the case of nulled position, velocity, or
    attitude, the corresponding error covariance terms are omitted. See also ECi coordinate frame
    definition for x, y, and z definitions.
    """
    ECI = 0

    """
    Sensor position, velocity, and attitude relative to the WGS-84 reference. Position is expressed
    where p1 is the geodetic latitude in radians, p2 is the longitude in radians, and p3 is the geodetic
    altitude (or height above the WGS-84 ellipsoid) in meters. Velocity with respect to the local level
    geographical frame with the origin at the sensor position is expressed using the North-East-Down
    (NED) convention, where v1 is the north velocity component in m/s, v2 is the east velocity component
    in m/s, and v3 is the down velocity component in m/s. Attitude is the sensor frame rotation relative
    to the local level frame expressed as a four-element quaternion. Position, in whole or in part, is
    optional. Velocity, in whole or in part, is optional. Attitude is optional, but if included, must be
    3 dimensional. Error covariance is given using north-south position error (meters), east-west
    position error (meters), geodetic altitude error (meters), north velocity component error (m/s),
    east velocity component error (m/s), down velocity component error (m/s), tilt error about the north
    axis (rad), tilt error about the east axis (rad), and tilt error about the down axis (rad). In the
    case of nulled position, velocity, or attitude, the corresponding error covariance terms are
    omitted.
    """
    GEODETIC = 1


class MeasurementPositionVelocityAttitudeErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementPositionVelocityAttitude(AspnBase):
    """
    Position, velocity, and attitude. Position, in whole or in part, is optional. Velocity, in whole
    or in part, is optional. Attitude is optional, but if included, must be 3-dimensional. Position is
    relative to a user-defined reference frame. Velocity is time rate of change of position with respect
    to the user-specified reference frame. Attitude expressed as a quaternion, a four element vector
    representation, the elements of which are functions of the orientation of the vector and the
    magnitude of the rotation.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    reference_frame - MeasurementPositionVelocityAttitudeReferenceFrame:
            Defines measurement reference.

    p1 - Optional[Optional[float]]:
            First position term as defined in enum.

    p2 - Optional[Optional[float]]:
            Second position term as defined in enum.

    p3 - Optional[Optional[float]]:
            Third position term as defined in enum.

    v1 - Optional[Optional[float]]:
            Velocity along the first axis of the measurement reference frame enumerated in
            reference_frame.

    v2 - Optional[Optional[float]]:
            Velocity along the second axis of the measurement reference frame enumerated in
            reference_frame.

    v3 - Optional[Optional[float]]:
            Velocity along the third axis of the measurement reference frame enumerated in
            reference_frame.

    quaternion - Optional[Optional[NumpyArray[float]]]:
            Four element quaternion, q = [a, b, c, d], where a = cos(phi/2), b =
            (phi_x/phi)*sin(phi/2), c = (phi_y/phi)*sin(phi/2), and d = (phi_z/phi)*sin(phi/2). In
            this description, the vector [phi_x, phi_y, phi_z] represents the rotation vector that
            describes the frame rotation to be applied to the "reference" frame (ECI, ECEF, or NED)
            to rotate it into the axes that describe the measured attitude, and the value phi is the
            magnitude of the [phi_x, phi_y, phi_z] vector. See "conventions" documentation for more
            detailed information.

    covariance - NumpyMatrix[float]:
            Measurement error variance or covariance depending on measurement dimension. NOTE:
            Attitude errors are expressed as tilt errors, so if an attitude is provided, that should
            count as 3 terms in num_meas. For example, a 3-D position, 3-D velocity, and attitude
            expressed as a quaternion (which takes 10 terms total to describe) is num_meas = 9. A
            second example is a 3-D position and 3-D velocity is num_meas = 6. Dimensions of
            covariance must be num_meas²

    error_model - MeasurementPositionVelocityAttitudeErrorModel:
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
    reference_frame: MeasurementPositionVelocityAttitudeReferenceFrame
    p1: Optional[float]
    p2: Optional[float]
    p3: Optional[float]
    v1: Optional[float]
    v2: Optional[float]
    v3: Optional[float]
    quaternion: Optional[NumpyArray[float]]
    covariance: NumpyMatrix[float]
    error_model: MeasurementPositionVelocityAttitudeErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
