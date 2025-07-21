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

from .type_header import TypeHeader


@dataclass
class TypeMetadataheader:
    """
    Sensor metadata header.

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    sensor_description - str:
            Describes the device used to generate the measurement (user defined).

    delta_t_nom - Optional[Optional[float]]:
            Optional nominal time interval between each measurement. Actual time interval may
            vary. Do not report a delta_t_nom if the nominal time interval is not known ahead of
            time.

    timestamp_clock_id - int:
            Identifier for the timestamp's clock/timing source. See clock_identifiers.md for a
            full description, but the summary is as follows: 0 = ASPN System Time 1 = International
            Atomic Time (TAI) 2 = Universal Coordinated Time (UTC) 3 = GPS System Time 4 = Galileo
            System Time 5 = GLONASS System Time 6 = BeiDou System Time 7-50: Reserved for future
            additional time scale representations. Each message source must provide metadata
            information (the method used to provide the metadata information depends on the system,
            e.g., over the wire, written to file, etc.), and the information contained in this type
            (type_metadataheader) will be included in that information. In this manner, each message
            source's clock ID will be defined. If the clock source is updated, new metadata must be
            provided with an updated clock ID. If used, ASPN system time shall be represented as a
            monotonically increasing quantity defined by the system. System time zero epoch must be
            defined. For example, system time may be the time difference with respect to a system
            power on event. Other user-defined clocks/timing sources must be similarly defined by
            the system designer.

    digits_of_precision - int:
            Defines how many decimal digits of precision are represented in the timestamp after
            the decimal point. The precision of the timestamps is 10E-N, where N is
            digits_of_precision. A value of 3 would mean that the timestamp has precision at the
            10E-3 (millisecond) level, a value of 6 would mean the timestamp has precision at the
            10E-6 (microsecond) level, etc. If an implementation is using 64bit integers for their
            internal time representations, digits of precision may be ignored with no additional
            complexity. If an implementation is converting to a double, because of their choice of
            internal representation (which likely most estimators would use), then knowing digits of
            precision would be important to avoid losing resolution.
    """

    header: TypeHeader
    sensor_description: str
    delta_t_nom: Optional[float]
    timestamp_clock_id: int
    digits_of_precision: int
