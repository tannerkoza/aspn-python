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
class TypeSatnavClock:
    """
    satnav broadcast parameters required to calculate sv clock corrections

    ### Attributes

    t_oc - float:
            Clock reference time.

    af_0 - float:
            Satellite clock bias.

    af_1 - float:
            Satellite clock drift.

    af_2 - float:
            Satellite clock drift rate.
    """

    t_oc: float
    af_0: float
    af_1: float
    af_2: float
