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

from .aspn_base import AspnBase
from .type_metadataheader import TypeMetadataheader
from .type_mounting import TypeMounting
from .type_timestamp import TypeTimestamp


class MetadataImuErrorModel(Enum):
    """
    Allows the user to select an appropriate error model. Parameters are defined here. The number of
    parameters and the values for those parameters are included in subsequent fields.
    """

    """
    All axes are identical and uncorrelated with each other axis (num_error_model_params = 14).
    1) accel_bias_sigma, units: m/s/s, description: Standard deviation of constant bias for
    accelerometer measurements.
    2) accel_bias_mean, units: m/s/s, description: Mean of constant bias for accelerometer
    measurements.
    3) accel_time_correlated_bias_sigma, units: m/s/s, description: Standard deviation of
    zero-mean time-correlated bias for accelerometer measurement.
    4) accel_time_correlated_bias_time_constant, units: s, description: Time constant of
    zero-mean time-correlated bias for accelerometer measurement.
    5) accel_scale_factor_mean, units: ppm, description: Mean of accelerometer scale factor
    modeled as a constant with Gaussian uncertainty.
    6) accel_scale_factor_sigma, units: ppm, description: Standard deviation of accelerometer
    scale factor modeled as a constant with Gaussian uncertainty.
    7) velocity_random_walk, units: m/s/sqrt(s), description: Velocity random walk due to
    acceleration.
    8) gyro_bias_sigma, units: rad/s, description: Standard deviation of constant bias for gyro
    measurements.
    9) gyro_bias_mean, units: rad/s, description: Mean of constant bias for gyro measurements.
    10) gyro_time_correlated_bias_sigma, units: rad/s, description: Standard deviation of
    zero-mean time-correlated bias for gyro measurements.
    11) gyro_time_correlated_bias_time_constant, units: s, description: Time constant of
    zero-mean time-correlated bias for gyro measurements.
    12) gyro_scale_factor_mean, units: ppm, description: Mean of gyro scale factor modeled as a
    constant with Gaussian uncertainty.
    13) gyro_scale_factor_sigma, units: ppm, description: Standard deviation of gyro scale
    factor modeled as a constant with Gaussian uncertainty.
    14) angular_random_walk, units: rad/sqrt(s), description: Angular random walk due to gyros.
    """
    BASIC = 0


@dataclass
class MetadataImu(AspnBase):
    """
    Metadata for inertial measurement unit.

    ### Attributes

    info - TypeMetadataheader:
            Standard ASPN metadata header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    mounting - TypeMounting:
            Standard ASPN mounting information.

    error_model - MetadataImuErrorModel:
            Allows the user to select an appropriate error model. Parameters are defined here.
            The number of parameters and the values for those parameters are included in subsequent
            fields.

    error_model_params - NumpyArray[float]:
            Error model parameters that characterize the optional error model.
    """

    info: TypeMetadataheader
    time_of_validity: TypeTimestamp
    mounting: TypeMounting
    error_model: MetadataImuErrorModel
    error_model_params: NumpyArray[float]
