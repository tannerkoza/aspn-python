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


class MeasurementAttitude3DReferenceFrame(Enum):
    """
    Defines measurement reference.
    """

    """
    Absolute inertial attitude. Sensor frame attitude relative to the earth-centered, inertial (ECi)
    frame. See also ECi coordinate frame definition for x, y, and z definitions.
    """
    ECI = 0

    """
    Attitude relative to Earth. Sensor frame attitude relative to the ECEF frame. Generally, time is
    required to convert between absolute inertial attitude and attitude relative to Earth. See also ECEF
    coordinate frame definition for x, y, and z definitions.
    """
    ECEF = 1

    """
    Local level, geographic attitude. Sensor frame attitude with respect to earth-fixed frame
    expressed in the North-East-Down (NED), local level frame. See also NED coordinate frame definition
    for N, E, and D definitions. Generally, position is required to convert between attitude relative to
    the earth and locally level, geographic attitude. See also NED coordinate frame definition for N, E,
    and D definitions.
    """
    NED = 2


class MeasurementAttitude3DErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementAttitude3D(AspnBase):
    """
    Three-dimensional attitude expressed as a quaternion, a four element vector representation, the
    elements of which are functions of the orientation of the vector and the magnitude of the rotation.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid .

    reference_frame - MeasurementAttitude3DReferenceFrame:
            Defines measurement reference.

    quaternion - NumpyArray[float]:
            Four element quaternion, q = [a, b, c, d], where a = cos(phi/2), b =
            (phi_x/phi)*sin(phi/2), c = (phi_y/phi)*sin(phi/2), and d = (phi_z/phi)*sin(phi/2). In
            this description, the vector [phi_x, phi_y, phi_z] represents the rotation vector that
            describes the frame rotation to be applied to the "reference" frame (ECI, ECEF, or NED)
            to rotate it into the axes that describe the measured attitude, and the value phi is the
            magnitude of the [phi_x, phi_y, phi_z] vector. See "conventions" documentation for more
            detailed information.

    tilt_error_covariance - NumpyMatrix[float]:
            Tilt error covariance matrix. This matrix represents the uncertainty in the "tilt
            errors" that represent the additional rotation to be applied to the provided attitude
            quaternion in order to convert it to the true attitude with no errors. By convention,
            these "tilt errors" are expressed in the reference frame (ECI, ECEF, or NED). See
            "conventions" documentation for more detailed information.

    error_model - MeasurementAttitude3DErrorModel:
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
    reference_frame: MeasurementAttitude3DReferenceFrame
    quaternion: NumpyArray[float]
    tilt_error_covariance: NumpyMatrix[float]
    error_model: MeasurementAttitude3DErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
