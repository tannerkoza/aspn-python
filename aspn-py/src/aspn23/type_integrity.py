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


class TypeIntegrityIntegrityMethod(Enum):
    """
    Enumerated field defines the integrity method used and the appropriate interpretation of
    integrity_value. Integrity methods may use multiple enum entries if there is a need to pass multiple
    integrity values. For example, an integrity method with 3 parameters may use 3 enum entries (e.g.,
    MY_METHOD_VALUE1, MY_METHOD_VALUE2, MY_METHOD_VALUE3). When integrity is reported in a message, a
    vector of integrity types is passed. In the MY_METHOD example, num_integrity = 3 just for the
    MY_METHOD integrity. If additional integrity methods are also reported, then num_integrity would be
    greater than 3 to include those additional methods.
    See extensibility.md for more information on implementing enums.
    """

    """
    Integrity method that identifies the assurance state of the measurement derived from the
    assurance status defined in IS-GPS-153E. Use the following values for integrity_values.
    integrity_value = 0, NOT_AVAILABLE. No assurance information available (default when
    receiver does not provide assurance status)
    integrity_value = 1, INIT.
    integrity_value = 2, PENDING.
    integrity_value = 3, USABLE.
    integrity_value = 4, ASSURED. Measurement is assured.
    integrity_value = 5, SUSPECT.
    """


@dataclass
class TypeIntegrity:
    """
    Integrity provides a numerical measure of the trustworthiness of the associated measurement. The
    integrity_method enum specifies the integrity method and the appropriate interpretation of the
    integrity_value.

    ### Attributes

    integrity_method - TypeIntegrityIntegrityMethod:
            Enumerated field defines the integrity method used and the appropriate
            interpretation of integrity_value. Integrity methods may use multiple enum entries if
            there is a need to pass multiple integrity values. For example, an integrity method with
            3 parameters may use 3 enum entries (e.g., MY_METHOD_VALUE1, MY_METHOD_VALUE2,
            MY_METHOD_VALUE3). When integrity is reported in a message, a vector of integrity types
            is passed. In the MY_METHOD example, num_integrity = 3 just for the MY_METHOD integrity.
            If additional integrity methods are also reported, then num_integrity would be greater
            than 3 to include those additional methods. See extensibility.md for more information on
            implementing enums.

    integrity_value - Optional[Optional[float]]:
            Measurement integrity value to be interpreted based on the definition in the
            integrity_type enum.
    """

    integrity_method: TypeIntegrityIntegrityMethod
    integrity_value: Optional[float]
