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
class MetadataGlonassEphemeris(AspnBase):
    """
    GLONASS Ephemeris describing GLONASS satellite locations. Definitions and usage are covered in
    GLONASS ICD L1,L2 - Edition 5.1 2008, Section 4.4.

    ### Attributes

    info - TypeMetadataheader:
            Standard ASPN metadata header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    slot_number - int:
            GLONASS satellite slot number.

    freq_o - int:
            Frequency channel offset number in range from 0 to 20.

    n_t - int:
            Calender number of day within 4 year interval starting at Jan 1 of a leap year

    t_k - float:
            Time referenced to the beginning of the frame within the current day.

    t_b - float:
            Index of time interval within current day according to UTC(SU) + 03 hrs.

    gamma_n - float:
            Relative Satellite frequency bias

    tau_n - float:
            Satellite clock bias.

    sv_pos_x - float:
            Satellite X position in PZ-90 coordinate system at time t_b.

    sv_vel_x - float:
            Satellite X velocity in PZ-90 coordinate system at time t_b.

    sv_accel_x - float:
            Satellite X acceleration in PZ-90 coordinate system at time t_b.

    sv_pos_y - float:
            Satellite Y position in PZ-90 coordinate system at time t_b.

    sv_vel_y - float:
            Satellite Y velocity in PZ-90 coordinate system at time t_b.

    sv_accel_y - float:
            Satellite Y acceleration in PZ-90 coordinate system at time t_b.

    sv_pos_z - float:
            Satellite Z position in PZ-90 coordinate system at time t_b.

    sv_vel_z - float:
            Satellite Z velocity in PZ-90 coordinate system at time t_b.

    sv_accel_z - float:
            Satellite Z acceleration in PZ-90 coordinate system at time t_b.
    """

    info: TypeMetadataheader
    time_of_validity: TypeTimestamp
    slot_number: int
    freq_o: int
    n_t: int
    t_k: float
    t_b: float
    gamma_n: float
    tau_n: float
    sv_pos_x: float
    sv_vel_x: float
    sv_accel_x: float
    sv_pos_y: float
    sv_vel_y: float
    sv_accel_y: float
    sv_pos_z: float
    sv_vel_z: float
    sv_accel_z: float
