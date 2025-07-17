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
from .type_timestamp import TypeTimestamp


@dataclass
class MetadataSatnavObs(AspnBase):
    """
    Metadata for satnav_obs

    ### Attributes

    info - TypeMetadataheader:
            Standard ASPN metadata header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    deltarange_interval_start - Optional[Optional[float]]:
            Start time of the interval over which the deltarange is measured with respect to the
            measurement time. This value must be provided if the deltarange measurement is provided.

    deltarange_interval_stop - Optional[Optional[float]]:
            Stop time of the interval over which the deltarange is measured with respect to the
            measurement time. This value must be provided if the deltarange measurement is provided.
            If the deltarange integration period is the same as the measurement time, this value
            should be 0.
    """

    info: TypeMetadataheader
    time_of_validity: TypeTimestamp
    deltarange_interval_start: Optional[float]
    deltarange_interval_stop: Optional[float]
