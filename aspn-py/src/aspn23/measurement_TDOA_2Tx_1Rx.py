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


class MeasurementTdoa2Tx1RxErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementTdoa2Tx1Rx(AspnBase):
    """
    Time difference of arrival of two signals (Tx) to a sensor (Rx).

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    tx1_position_and_covariance - TypeRemotePoint:
            Transmitter 1 (Tx1)'s known location.

    tx2_position_and_covariance - TypeRemotePoint:
            Transmitter 2 (Tx2)'s known location.

    obs - float:
            Time difference of arrival (in meters). Time difference of Tx1 signal's arrival time
            minus Tx2 signal's arrival time at the sensor. Positive obs indicates Tx1 arrived later
            than Tx2.

    variance - float:
            Measurement variance.

    error_model - MeasurementTdoa2Tx1RxErrorModel:
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
    tx1_position_and_covariance: TypeRemotePoint
    tx2_position_and_covariance: TypeRemotePoint
    obs: float
    variance: float
    error_model: MeasurementTdoa2Tx1RxErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
