"""
This code is generated via https://git.aspn.us/pntos/firehose/-/blob/main/firehose/backends/aspn/aspn_yaml_to_python.py
DO NOT hand edit code.  Make any changes required using the firehose repo instead.
"""

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

from .type_satnav_time import TypeSatnavTime


class TypeNavsimSatnavSvDataEphemerisType(Enum):
    """
    Provides further clarification of which ephemeris was used to generate the satellite information
    provided in this message for systems which have more than one ephemeris representation.
    """

    """
    To be used when a system only uses one ephemeris representation, so knowing the satellite_system
    fully defines what ephemeris is being used.
    """
    SET_BY_SYSTEM = 0

    """
    Calculated using legacy GPS (LNAV) messages
    """
    GPS_LNAV = 1

    """
    Calculated using GPS CNAV messages
    """
    GPS_CNAV = 2

    """
    Calculated using GPS MNAV messages
    """
    GPS_MNAV = 3


class TypeNavsimSatnavSvDataCoordinateFrame(Enum):
    """
    Coordinate frame system used to describe satellite position and velocity.
    """

    """
    International Terrestrial Reference Frame
    """
    ITRF = 0

    """
    Earth-centered earth-fixed frame as defined by WGS-84 (nearly identical with ITRF)
    """
    ECEF = 1

    """
    Galileo Terrestrial Reference Frame (nearly identical with ITRF)
    """
    GTRF = 2

    """
    Parametri Zemli 1990 (PZ-90) reference frame
    """
    PZ90 = 3


class TypeNavsimSatnavSvDataGroupDelayEnum(Enum):
    """
    Describes how to interpret group delay terms, because they vary by ephemeris type. Descriptions
    assume zero-indexing.
    """

    """
    group_delay_vector[0] is legacy Tgd as defined in ICD-GPS-200L Section 20.3.3.3.3.2. All other
    terms not used.
    """
    TGD_LNAV = 0


@dataclass
class TypeNavsimSatnavSvData:
    """
    Satellite position, velocity and clock error at a particular time epoch

    ### Attributes

    prn - str:
            PRN code which identifies satellite (or slot number, in the case of GLONASS) of this
            ephemeris.

    satellite_system - str:
            Describes the Satellite System which was used to generate the satellite information
            provided in this message.

    ephemeris_type - TypeNavsimSatnavSvDataEphemerisType:
            Provides further clarification of which ephemeris was used to generate the satellite
            information provided in this message for systems which have more than one ephemeris
            representation.

    sv_data_time - TypeSatnavTime:
            Receiver time at which the data provided in this message is valid.

    coordinate_frame - TypeNavsimSatnavSvDataCoordinateFrame:
            Coordinate frame system used to describe satellite position and velocity.

    sv_pos - NumpyArray[float]:
            Satellite position in frame specified by coordinate_frame at time specified by
            sv_data_time_week_number and sv_data_time_seconds_of_week.

    sv_vel - NumpyArray[float]:
            Satellite velocity in frame specified by coordinate_frame at time specified by
            sv_data_time_week_number and sv_data_time_seconds_of_week.

    sv_clock_bias - float:
            Satellite clock bias at the sv_data_time, used to correct the satellite time like
            delta_t_sv in equation (1) of ICD-GPS-200L, Section 20.3.3.3.3.1.

    sv_clock_drift - float:
            Satellite clock drift rate at the sv_data_time. By way of example, for GPS this
            would be af1 in equation (2) of ICD-GPS-200L, Section 20.3.3.3.3.1.

    group_delay_enum - TypeNavsimSatnavSvDataGroupDelayEnum:
            Describes how to interpret group delay terms, because they vary by ephemeris type.
            Descriptions assume zero-indexing.

    group_delay_vector - NumpyArray[float]:
            Group delay terms, with interpretation provided by group_delay_enum. (Need to verify
            that four is sufficient for all ephemeris.)
    """

    prn: str
    satellite_system: str
    ephemeris_type: TypeNavsimSatnavSvDataEphemerisType
    sv_data_time: TypeSatnavTime
    coordinate_frame: TypeNavsimSatnavSvDataCoordinateFrame
    sv_pos: NumpyArray[float]
    sv_vel: NumpyArray[float]
    sv_clock_bias: float
    sv_clock_drift: float
    group_delay_enum: TypeNavsimSatnavSvDataGroupDelayEnum
    group_delay_vector: NumpyArray[float]
