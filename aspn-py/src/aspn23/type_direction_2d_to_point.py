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

from .type_image_feature import TypeImageFeature
from .type_integrity import TypeIntegrity
from .type_remote_point import TypeRemotePoint


class TypeDirection2DToPointReference(Enum):
    """
    Specifies measurement reference.
    """

    """
    Angle between the sensor's local level N-axis and the vector from the sensor to the point
    projected onto the sensor's local level North-East plane. Positive angle is defined as a rotation
    towards the E-axis.
    See also NED coordinate frame definition for N, E, and D definitions. A figure depicting the
    angle definition is included in the conventions documentation.
    """
    NE_TO = 0

    """
    Angle between the point's local level N-axis and the vector from the point to the sensor
    projected onto the point's local level North-East plane. Positive angle is defined as a rotation
    towards the E-axis.
    The significant difference between NE_TO and NE_FROM is where the NED frame originates (NE_FROM
    has its origin located at the point's location).
    See also NED coordinate frame definition for N, E, and D definitions. A figure depicting the
    angle definition is included in the conventions documentation.
    """
    NE_FROM = 1

    """
    Angle between the sensor's local level North-East plane and the vector from the sensor to the
    point. Positive angle is defined as a rotation towards the negative D-axis of the NED frame.
    See also NED coordinate frame definition for N, E, and D definitions. A figure depicting the
    angle definition is included in the conventions documentation.
    """
    ELEVATION = 2

    """
    Angle between the sensor's x-axis and the vector from the sensor to the point projected onto the
    sensor frame x-y plane. Positive angle is defined as a rotation towards the y-axis.
    Sensor frame is defined in mounting. A figure depicting the angle definition is included in the
    conventions documentation.
    """
    SENSOR = 3


class TypeDirection2DToPointErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class TypeDirection2DToPoint:
    """
    2D direction to or from a point as defined by the reference enum. Information about the remote
    point (its position and/or correspondence with previous or future appearances of this same point)
    may be determined using remote_point.

    ### Attributes

    remote_point - TypeRemotePoint:
            Position and/or correspondence information about the remote point.

    reference - TypeDirection2DToPointReference:
            Specifies measurement reference.

    obs - float:
            Observations as specified in the reference enum.

    variance - float:
            Measurement error variance.

    has_observation_characteristics - bool:
            Switch for whether observation_characteristics is valid or not.

    observation_characteristics - TypeImageFeature:
            Image feature characteristics.

    error_model - TypeDirection2DToPointErrorModel:
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

    remote_point: TypeRemotePoint
    reference: TypeDirection2DToPointReference
    obs: float
    variance: float
    has_observation_characteristics: bool
    observation_characteristics: TypeImageFeature
    error_model: TypeDirection2DToPointErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
