"""
This code is generated via https://git.aspn.us/pntos/firehose/-/blob/main/firehose/backends/aspn/aspn_yaml_to_python.py
DO NOT hand edit code.  Make any changes required using the firehose repo instead.
"""

from dataclasses import dataclass
from enum import Enum
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
from .type_remote_point import TypeRemotePoint
from .type_timestamp import TypeTimestamp


class MeasurementTdoa1Tx2RxErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementTdoa1Tx2Rx(AspnBase):
    """
    Time difference of arrival of a single transmitter (Tx) to two receivers. Receiver 1 (Rx1) is at
    a known location. Receiver 2 is the sensor system providing the measurement.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    tx_position_and_covariance - TypeRemotePoint:
            Transmitter (Tx)'s known location.

    rx1_position_and_covariance - TypeRemotePoint:
            Receiver 1 (Rx1)'s known location.

    obs - float:
            Time difference of arrival (in meters). Time difference of Tx signal's arrival time
            at Rx1 minus Tx signal's arrival time at the sensor. Positive obs indicates Tx arrived
            at Rx1 later than arriving at the sensor.

    variance - float:
            Measurement variance.

    error_model - MeasurementTdoa1Tx2RxErrorModel:
            Defines an optional error model for other than zero-mean, additive, white Gaussian
            noise (AWGN).

    error_model_params - NumpyArray[float]:
            Error model parameters that characterize the optional error model.

    integrity - List[TypeIntegrity]:
            Measurement integrity. Includes the integrity method used and an integrity value
            (which is to be interpreted based upon the integrity method). The intent of allowing
            num_integrity > 1 is to report multiple integrity values based on multiple integrity
            methods.
    """

    header: TypeHeader
    time_of_validity: TypeTimestamp
    tx_position_and_covariance: TypeRemotePoint
    rx1_position_and_covariance: TypeRemotePoint
    obs: float
    variance: float
    error_model: MeasurementTdoa1Tx2RxErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
