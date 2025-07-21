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


class MeasurementHeadingReference(Enum):
    """
    Defines heading reference.
    """

    """
    True heading. Direction to the geographic north expressed in the locally level NED frame as a
    rotation about the D-axis. This is akin to attitude_1d, which would be consistent with attitude_2d
    and attitude_3d. In that case, a ref frame enum could be used to define multiple one-dimensional
    attitudes, of which, heading could be one. The simpler approach of just having heading was chosen
    over the more general approach until there is a requirement for multiple, one-dimensional attitudes.
    """
    TRUE = 0

    """
    Magnetic heading. Direction to magnetic north expressed in the locally level NED frame as a
    rotation about the D-axis. In general, geographic position and time is required to convert from
    magnetic heading to true heading.
    """
    MAGNETIC = 1


class MeasurementHeadingErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementHeading(AspnBase):
    """
    Local level, geographic attitude expressed in the locally level NED frame as a rotation about
    the D-axis. Positive rotation follows the right-hand rule. Attitude about the other axes in the
    NED-frame are unspecified and not sensed. Use attitude_2d or attitude_3d to report a sensed
    dimension that is zero. See also NED coordinate frame definition for N, E, and D definitions.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    reference - MeasurementHeadingReference:
            Defines heading reference.

    obs - float:
            Heading.

    variance - float:
            Variance of the measurement.

    error_model - MeasurementHeadingErrorModel:
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
    reference: MeasurementHeadingReference
    obs: float
    variance: float
    error_model: MeasurementHeadingErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
