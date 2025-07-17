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
from .type_header import TypeHeader
from .type_integrity import TypeIntegrity
from .type_satnav_obs import TypeSatnavObs
from .type_satnav_time import TypeSatnavTime
from .type_timestamp import TypeTimestamp


@dataclass
class MeasurementSatnav(AspnBase):
    """
    Raw measurements from a satnav receiver.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    receiver_clock_time - TypeSatnavTime:
            Receiver clock time. In a multi-GNSS receiver (GPS/GLONASS/Galileo/QZSS/BeiDou) all
            pseudorange observations must refer to one receiver clock only. The receiver clock time
            of the measurement is the receiver clock time of the received signals. It is identical
            for the phase and range measurements and is identical for all satellites observed in a
            given epoch.

    num_signal_types - int:
            Number of different signal types tracked in the current epoch. Examples of signal
            types include GPS L1 C/A code, Galileo E1B.

    obs - List[TypeSatnavObs]:
            Array of satnav obs data for all of the signals/PRNs being tracked in this epoch.

    integrity - List[TypeIntegrity]:
            Measurement integrity at the sensor level. Integrity is also available for each
            observable, which is found in the observable type definition. Includes the integrity
            method used and an integrity value (which is to be interpreted based upon the integrity
            method). The intent of allowing num_integrity > 1 is to report multiple integrity values
            based on multiple integrity methods.
    """

    header: TypeHeader
    time_of_validity: TypeTimestamp
    receiver_clock_time: TypeSatnavTime
    num_signal_types: int
    obs: List[TypeSatnavObs]
    integrity: List[TypeIntegrity]
