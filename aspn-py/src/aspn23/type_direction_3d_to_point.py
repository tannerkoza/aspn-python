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


class TypeDirection3DToPointReferenceFrame(Enum):
    """
    Defines measurement reference frame.
    """

    """
    Azimuth (first component of obs) is the angle between the sensor frame x-axis (boresight) and
    the projection of the point onto the sensor frame's x-y plane (positive right-hand rule about the
    sensor frame z-axis). Elevation (second component of obs) is the angle between the sensor frame's
    x-y plane and the vector from the sensor to the point (positive right-hand rule about the
    intermediate axis formed by the azimuth rotation). The covariance is for the azimuth and elevation
    errors. Units for the observation are in rad, covariance rad^2.
    """
    AZ_EL = 0

    """
    The pixel coordinates of the point in an image (sub-pixel resolution is allowed). When combined
    with the camera model, these can be converted to the NORMALIZED_IMAGE reference frame
    representation. Units for the observation are in pixels, covariance pixels^2.
    """
    PIXEL = 1

    """
    2D position from the sensor frame origin to the point expressed in components y and z of the
    sensor frame normalized by the boresight distance. The x component of the sensor frame is the
    boresight axis. Units for the observation are m/m, covariance (m/m)^2.
    """
    NORMALIZED_IMAGE = 2

    """
    2D position from the sensor frame origin to the point expressed in components y and z of the
    sensor frame normalized by the range to the point. The x component of the sensor frame is the
    boresight axis. Units for the observation are m/m, covariance (m/m)^2.
    """
    SINE_SPACE = 3


class TypeDirection3DToPointErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class TypeDirection3DToPoint:
    """
    3D direction to or from a point as defined by the reference enum. Information about the remote
    point (its position and/or correspondence with previous or future appearances of this same point)
    may be determined using remote_point.

    ### Attributes

    remote_point - TypeRemotePoint:
            Position and/or correspondence information about the remote point.

    reference_frame - TypeDirection3DToPointReferenceFrame:
            Defines measurement reference frame.

    obs - NumpyArray[float]:
            3D direction to the remote point as defined in reference_frame.

    covariance - NumpyMatrix[float]:
            Covariance of the direction measurement as defined in reference_frame.

    has_observation_characteristics - bool:
            Switch for whether observation_characteristics is valid or not.

    observation_characteristics - TypeImageFeature:
            Image feature characteristics.

    error_model - TypeDirection3DToPointErrorModel:
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
    reference_frame: TypeDirection3DToPointReferenceFrame
    obs: NumpyArray[float]
    covariance: NumpyMatrix[float]
    has_observation_characteristics: bool
    observation_characteristics: TypeImageFeature
    error_model: TypeDirection3DToPointErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
