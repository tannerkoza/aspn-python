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

from .aspn_base import AspnBase
from .type_metadataheader import TypeMetadataheader
from .type_mounting import TypeMounting
from .type_timestamp import TypeTimestamp


@dataclass
class MetadataMagneticField(AspnBase):
    """
    Metadata for a magnetic field measurement. In addition to generic metadata information,
    calibration parameters may be provided to account for effects such as soft iron, scale factor,
    non-orthogonality, zero-bias, and hard iron. In general, for an num_meas-dimensional measurement,
    the magnetic field calibration metadata (K and b) shall be used as m_calibrated = K * m_measured - b
    where m_calibrated, m_measured, and b are num_meas x 1 vectors (scalar for num_meas = 1) and K is an
    num_meas x num_meas matrix (scalar for num_meas = 1). See magnetic_calibration.md for additional
    details. More sophisticated approaches that include calibration parameters such as time-varying
    effects, first-order Gauss-Markov bias models, and calibration parameter uncertainties may be
    included using an appropriate error model in the measurement message.

    ### Attributes

    info - TypeMetadataheader:
            Standard ASPN metadata header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    mounting - TypeMounting:
            Standard ASPN mounting information.

    k - Optional[Optional[NumpyMatrix[float]]]:
            Optional calibration parameter to account for the combined effects of soft iron,
            scale factor, and non-orthogonality as a unitless num_meas x num_meas matrix. Optional,
            but if provided, b must also be provided.

    b - Optional[Optional[NumpyArray[float]]]:
            Optional calibration parameter to account for the combined effects of zero-bias and
            hard iron as a num_meas x 1 vector in nanoTesla (nT). Optional, but if provided, K must
            also be provided.
    """

    info: TypeMetadataheader
    time_of_validity: TypeTimestamp
    mounting: TypeMounting
    k: Optional[NumpyMatrix[float]]
    b: Optional[NumpyArray[float]]
