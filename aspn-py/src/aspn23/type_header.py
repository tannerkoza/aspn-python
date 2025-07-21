"""
This code is generated via https://git.aspn.us/pntos/firehose/-/blob/main/firehose/backends/aspn/aspn_yaml_to_python.py
DO NOT hand edit code.  Make any changes required using the firehose repo instead.
"""

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
class TypeHeader:
    """
    Header for each ASPN measurement

    ### Attributes

    vendor_id - int:
            Unique identifier that identifies the device or application vendor. Vendor ID is
            user-selected, inspired by your company name to mitigate conflicts with other users.
            Vendor IDs 0x23 00 00 00 through 0x23 FF FF FF inclusive are reserved and shall not be
            chosen as user-selected vendor IDs.

    device_id - int:
            Unique identifier that identifies the sensor or device type as assigned by the
            vendor. Device_ids are unique within a given vendor_id.

    context_id - int:
            Unique identifier that provides additional context to define a logical stream of
            data from the sensor or device as assigned by the vendor. This identifier allows
            multiple measurements of the same type to be provided by a device. Context_ids are
            unique within a given vendor_id and device_id.

    sequence_id - int:
            Unique identifier for a specific message within a data stream as defined by a
            vendor_id, device_id, and context_id. Sequential messages from each data source
            (identified by vendor_id, device_id, and context_id) shall increment by exactly 1 and
            rollover to 0 after an overflow.
    """

    vendor_id: int
    device_id: int
    context_id: int
    sequence_id: int
