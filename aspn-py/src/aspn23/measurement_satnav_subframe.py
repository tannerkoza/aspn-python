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
from .type_satnav_satellite_system import TypeSatnavSatelliteSystem
from .type_timestamp import TypeTimestamp


@dataclass
class MeasurementSatnavSubframe(AspnBase):
    """
    satnav navigation data bit subframe.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    gnss_message_id - int:
            Message ID provided from the satnav receiver.

    prn - int:
            Satellite ID number.

    satellite_system - TypeSatnavSatelliteSystem:
            Describes the satellite system that generated the subframe.

    freq_slot_id - int:
            Frequency slot +7 (range from 0-13). Only used for GLONASS.

    data_vector - NumpyArray[int]:
            num_bytes sized array of raw subframe message collected by the sensor. The
            underlying type and shape of the data vector is given by satnav_msg_id.

    integrity - List[TypeIntegrity]:
            Measurement integrity. Includes the integrity method used and an integrity value
            (which is to be interpreted based upon the integrity method). The intent of allowing
            num_integrity > 1 is to report multiple integrity values based on multiple integrity
            methods.
    """

    header: TypeHeader
    time_of_validity: TypeTimestamp
    gnss_message_id: int
    prn: int
    satellite_system: TypeSatnavSatelliteSystem
    freq_slot_id: int
    data_vector: NumpyArray[int]
    integrity: List[TypeIntegrity]
