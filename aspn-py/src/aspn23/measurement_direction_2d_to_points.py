from dataclasses import dataclass
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
from .type_direction_2d_to_point import TypeDirection2DToPoint
from .type_header import TypeHeader
from .type_timestamp import TypeTimestamp


@dataclass
class MeasurementDirection2DToPoints(AspnBase):
    """
    2D direction to points.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    obs - List[TypeDirection2DToPoint]:
            Array of observations.
    """

    header: TypeHeader
    time_of_validity: TypeTimestamp
    obs: List[TypeDirection2DToPoint]
