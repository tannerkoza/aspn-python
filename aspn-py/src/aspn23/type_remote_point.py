from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

# Backwards compatibility for typing numpy arrays
NumpyArray = np.ndarray  # represents a 1 dimensional numpy array
NumpyMatrix = np.ndarray  # represents a 2 dimensional numpy array
try:
    from numpy.typing import NDArray

    NumpyArray = NumpyMatrix = NDArray
except ImportError:
    pass


class TypeRemotePointPositionReferenceFrame(Enum):
    """
    Defines position reference.
    """

    """
    No reference frame. This is only valid if position is not included
    (included_terms is 0 or 1).
    """
    NONE = 0

    """
    Point position relative to the WGS-84 reference expressed using position1 as geodetic latitude
    in radians, position2 as longitude in radians, and position3 as geodetic altitude (or height
    above the WGS-84 ellipsoid) in meters. Each term is optional.
    position_covariance is given using north-south position error in meters (an expression of
    geodetic latitude error), east-west position error in meters (an expression of longitude
    error), and geodetic altitude error in meters. In the case of nulled position components, the
    corresponding error covariance terms are omitted.
    """
    GEODETIC = 1


@dataclass
class TypeRemotePoint:
    """
    ASPN custom type representing a point. This includes optional information (indicated by
    included_terms) for the position and a correspondence ID for the point.

    ### Attributes

    included_terms - int:
            Indicates which information is included about the point 0 = no information included
            (unknown point) 1 = id included (corresponded point) 2 = position included (known point)
            3 = id and position included

    id - int:
            Unique identification number assigned by the sensor. Points that have the same id
            from a sensor are multiple instances of the same point as determined by a sensor's point
            correspondence.

    position_reference_frame - TypeRemotePointPositionReferenceFrame:
            Defines position reference.

    position1 - Optional[Optional[float]]:
            First position term as defined in position_reference_frame.

    position2 - Optional[Optional[float]]:
            Second position term as defined in position_reference_frame.

    position3 - Optional[Optional[float]]:
            Third position term as defined in position_reference_frame.

    position_covariance - NumpyMatrix[float]:
            Position error covariance (or variance depending on num_position_components) as
            defined in position_reference_frame.
    """

    included_terms: int
    id: int
    position_reference_frame: TypeRemotePointPositionReferenceFrame
    position1: Optional[float]
    position2: Optional[float]
    position3: Optional[float]
    position_covariance: NumpyMatrix[float]
