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


class MeasurementAttitude2DReferenceFrame(Enum):
    """
    Defines measurement reference.
    """

    """
    Local level, geographic attitude expressed in the locally level NED frame as a rotation about
    the E-axis followed by rotation about the N-axis. Positive rotation follows the right-hand rule. The
    third rotation in the NED-frame is unspecified and is not sensed. Use attitude_3d to report a sensed
    third dimension that is zero. See also NED coordinate frame definition for N, E, and D definitions.
    """
    NE = 0

    """
    Local level, geographic attitude expressed in the locally level NED frame as a rotation about
    the D-axis followed by rotation about the N-axis. Positive rotation follows the right-hand rule. The
    third rotation in the NED-frame is unspecified and is not sensed. Use attitude_3d to report a sensed
    third dimension that is zero. See also NED coordinate frame definition for N, E, and D definitions.
    """
    ND = 1

    """
    Local level, geographic attitude expressed in the locally level NED frame as a rotation about
    the D-axis followed by rotation about the E-axis. Positive rotation follows the right-hand rule. The
    third rotation in the NED-frame is unspecified and is not sensed. Use attitude_3d to report a sensed
    third dimension that is zero. See also NED coordinate frame definition for N, E, and D definitions.
    """
    ED = 2


class MeasurementAttitude2DErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementAttitude2D(AspnBase):
    """
    Two-dimensional, local level, geographic attitude expressed in the locally level NED frame as a
    rotation about two axes as defined by the reference_frame.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid .

    reference_frame - MeasurementAttitude2DReferenceFrame:
            Defines measurement reference.

    attitude2d - NumpyArray[float]:
            Two-dimensional, local level, geographic attitude.

    covariance - NumpyMatrix[float]:
            Covariance of the measurement.

    error_model - MeasurementAttitude2DErrorModel:
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
    reference_frame: MeasurementAttitude2DReferenceFrame
    attitude2d: NumpyArray[float]
    covariance: NumpyMatrix[float]
    error_model: MeasurementAttitude2DErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
