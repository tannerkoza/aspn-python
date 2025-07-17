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


@dataclass
class TypeTimestamp:
    """
    Whole number nanoseconds elapsed since timestamp's zero epoch. If negative, whole number
    nanoseconds until timestamp's zero epoch. The zero epoch definition is based on the timing source
    used by the message provider. The timing source used by the message provider is defined by the
    `timestamp_clock_id` included in the `type_metadataheader` as part of the metadata. Additionally,
    the timestamp's digits of precision are included in the `type_metadataheader` as part of the
    metadata, which may be useful if an implementation is converting the timestamp to another type
    internally. If needed, the time elapsed in seconds is equal to elapsed_nsec * 1e-9.

    ### Attributes

    elapsed_nsec - int:
            Whole number nanoseconds elapsed since timestamp's zero epoch. If negative, whole
            number nanoseconds until timestamp's zero epoch.
    """

    elapsed_nsec: int
