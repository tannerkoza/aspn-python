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
from .type_timestamp import TypeTimestamp


class MeasurementMagneticFieldErrorModel(Enum):
    """
    Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
    """

    """
    No additional error model provided (num_error_model_params = 0).
    """
    NONE = 0


@dataclass
class MeasurementMagneticField(AspnBase):
    """
    Vector magnetic field. Represents the magnetic field strength along sensor x, y, and z axes as
    defined in mounting. May represent 1-D, 2-D, or 3-D measurement

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    x_field_strength - Optional[Optional[float]]:
            Field strength of magnetic field in nanoTesla (nT).

    y_field_strength - Optional[Optional[float]]:
            Field strength of magnetic field in nanoTesla (nT).

    z_field_strength - Optional[Optional[float]]:
            Field strength of magnetic field in nanoTesla (nT).

    covariance - NumpyMatrix[float]:
            Measurement error variance or covariance depending on measurement dimension.
            Dimensions of covariance must be num_meas²

    error_model - MeasurementMagneticFieldErrorModel:
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
    x_field_strength: Optional[float]
    y_field_strength: Optional[float]
    z_field_strength: Optional[float]
    covariance: NumpyMatrix[float]
    error_model: MeasurementMagneticFieldErrorModel
    error_model_params: NumpyArray[float]
    integrity: List[TypeIntegrity]
