from dataclasses import dataclass
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

from .aspn_base import AspnBase
from .type_metadataheader import TypeMetadataheader
from .type_mounting import TypeMounting
from .type_timestamp import TypeTimestamp


@dataclass
class MetadataGeneric(AspnBase):
    """
    Metadata for a generic sensor.

    ### Attributes

    info - TypeMetadataheader:
            Standard ASPN metadata header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    mounting - TypeMounting:
            Standard ASPN mounting information.
    """

    info: TypeMetadataheader
    time_of_validity: TypeTimestamp
    mounting: TypeMounting
