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

from .aspn_base import AspnBase
from .type_metadataheader import TypeMetadataheader
from .type_timestamp import TypeTimestamp


@dataclass
class MetadataGpsIonoUtcParameters(AspnBase):
    """
    Broadcasted GPS navigation data for estimation of single frequency ionospheric correction and
    determination of UTC. Definitions and usage are covered in ICD-GPS-200L, Section 20.3.3.5.1.6 and
    20.3.3.5.1.7.

    ### Attributes

    info - TypeMetadataheader:
            Standard ASPN metadata header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    a_0 - float:
            Bias coefficient of GPS time scale relative to UTC time scale.

    a_1 - float:
            Drift coefficient of GPS time scale relative to UTC time scale.

    delta_t_ls - int:
            Leap second count.

    tot - int:
            Reference GPS time of week for UTC parameters

    wn_t - int:
            Reference Modulo 256 GPS Week number for UTC parameters

    wn_lsf - int:
            Modulo 256 GPS Week number at the end of which delta_t_lsf becomes effective.

    dn - int:
            Day number at the end of which delta_t_lsf becomes effective (1 to 7).

    delta_t_lsf - int:
            Future leap second count.

    alpha_0 - float:
            Zeroth-order coefficient of amplitude of vertical ionospheric delay.

    alpha_1 - float:
            First-order coefficient of amplitude of vertical ionospheric delay.

    alpha_2 - float:
            Second-order coefficient of amplitude of vertical ionospheric delay.

    alpha_3 - float:
            Third-order coefficient of amplitude of vertical ionospheric delay.

    beta_0 - float:
            Zeroth-order coefficient of period of ionospheric delay model.

    beta_1 - float:
            First-order coefficient of period of ionospheric delay model.

    beta_2 - float:
            Second-order coefficient of period of ionospheric delay model.

    beta_3 - float:
            Third-order coefficient of period of ionospheric delay model.
    """

    info: TypeMetadataheader
    time_of_validity: TypeTimestamp
    a_0: float
    a_1: float
    delta_t_ls: int
    tot: int
    wn_t: int
    wn_lsf: int
    dn: int
    delta_t_lsf: int
    alpha_0: float
    alpha_1: float
    alpha_2: float
    alpha_3: float
    beta_0: float
    beta_1: float
    beta_2: float
    beta_3: float
