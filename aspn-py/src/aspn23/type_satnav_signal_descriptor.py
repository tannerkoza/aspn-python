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


class TypeSatnavSignalDescriptorSignalDescriptor(Enum):
    """
    Three letter RINEX Observation Code of signal as given in Table A23 of RINEX 3.05 specification
    (https://files.igs.org/pub/data/format/rinex305.pdf). RINEX Observation Code must be paired with
    Satellite system to determine a unique identifier
    """

    """
    See RINEX 3.05 Table A23
    """
    L1C = 0

    """
    See RINEX 3.05 Table A23
    """
    L1S = 1

    """
    See RINEX 3.05 Table A23
    """
    L1L = 2

    """
    See RINEX 3.05 Table A23
    """
    L1X = 3

    """
    See RINEX 3.05 Table A23
    """
    L1P = 4

    """
    See RINEX 3.05 Table A23
    """
    L1W = 5

    """
    See RINEX 3.05 Table A23
    """
    L1N = 6

    """
    See RINEX 3.05 Table A23
    """
    L2C = 7

    """
    See RINEX 3.05 Table A23
    """
    L2D = 8

    """
    See RINEX 3.05 Table A23
    """
    L2S = 9

    """
    See RINEX 3.05 Table A23
    """
    L2L = 10

    """
    See RINEX 3.05 Table A23
    """
    L2X = 11

    """
    See RINEX 3.05 Table A23
    """
    L2P = 12

    """
    See RINEX 3.05 Table A23
    """
    L2W = 13

    """
    See RINEX 3.05 Table A23
    """
    L2N = 14

    """
    See RINEX 3.05 Table A23
    """
    L5I = 15

    """
    See RINEX 3.05 Table A23
    """
    L5Q = 16

    """
    See RINEX 3.05 Table A23
    """
    L5X = 17

    """
    See RINEX 3.05 Table A23
    """
    L4A = 18

    """
    See RINEX 3.05 Table A23
    """
    L4B = 19

    """
    See RINEX 3.05 Table A23
    """
    L4X = 20

    """
    See RINEX 3.05 Table A23
    """
    L6A = 21

    """
    See RINEX 3.05 Table A23
    """
    L6B = 22

    """
    See RINEX 3.05 Table A23
    """
    L6X = 23

    """
    See RINEX 3.05 Table A23
    """
    L3I = 24

    """
    See RINEX 3.05 Table A23
    """
    L3Q = 25

    """
    See RINEX 3.05 Table A23
    """
    L3X = 26

    """
    See RINEX 3.05 Table A23
    """
    L1B = 27

    """
    See RINEX 3.05 Table A23
    """
    L7I = 28

    """
    See RINEX 3.05 Table A23
    """
    L7Q = 29

    """
    See RINEX 3.05 Table A23
    """
    L7X = 30

    """
    See RINEX 3.05 Table A23
    """
    L8I = 31

    """
    See RINEX 3.05 Table A23
    """
    L8Q = 32

    """
    See RINEX 3.05 Table A23
    """
    L8X = 33

    """
    See RINEX 3.05 Table A23
    """
    L6C = 34

    """
    See RINEX 3.05 Table A23
    """
    L1Z = 35

    """
    See RINEX 3.05 Table A23
    """
    L5D = 36

    """
    See RINEX 3.05 Table A23
    """
    L5P = 37

    """
    See RINEX 3.05 Table A23
    """
    L5Z = 38

    """
    See RINEX 3.05 Table A23
    """
    L6S = 39

    """
    See RINEX 3.05 Table A23
    """
    L6L = 40

    """
    See RINEX 3.05 Table A23
    """
    L6E = 41

    """
    See RINEX 3.05 Table A23
    """
    L6Z = 42

    """
    See RINEX 3.05 Table A23
    """
    L2I = 43

    """
    See RINEX 3.05 Table A23
    """
    L2Q = 44

    """
    See RINEX 3.05 Table A23
    """
    L1D = 45

    """
    See RINEX 3.05 Table A23
    """
    L7D = 46

    """
    See RINEX 3.05 Table A23
    """
    L7P = 47

    """
    See RINEX 3.05 Table A23
    """
    L7Z = 48

    """
    See RINEX 3.05 Table A23
    """
    L8D = 49

    """
    See RINEX 3.05 Table A23
    """
    L8P = 50

    """
    See RINEX 3.05 Table A23
    """
    L6I = 51

    """
    See RINEX 3.05 Table A23
    """
    L6Q = 52

    """
    See RINEX 3.05 Table A23
    """
    L6D = 53

    """
    See RINEX 3.05 Table A23
    """
    L6P = 54

    """
    See RINEX 3.05 Table A23
    """
    L5A = 55

    """
    See RINEX 3.05 Table A23
    """
    L5B = 56

    """
    See RINEX 3.05 Table A23
    """
    L5C = 57

    """
    See RINEX 3.05 Table A23
    """
    L9A = 58

    """
    See RINEX 3.05 Table A23
    """
    L9B = 59

    """
    See RINEX 3.05 Table A23
    """
    L9C = 60

    """
    See RINEX 3.05 Table A23
    """
    L9X = 61

    """
    See RINEX 3.05 Table A23
    """
    L1Y = 62

    """
    See RINEX 3.05 Table A23
    """
    L1M = 63

    """
    See RINEX 3.05 Table A23
    """
    L2Y = 64

    """
    See RINEX 3.05 Table A23
    """
    L2M = 65


@dataclass
class TypeSatnavSignalDescriptor:
    """
    Satellite signal descriptor as defined in RINEX 3.05

    ### Attributes

    signal_descriptor - TypeSatnavSignalDescriptorSignalDescriptor:
            Three letter RINEX Observation Code of signal as given in Table A23 of RINEX 3.05
            specification (https://files.igs.org/pub/data/format/rinex305.pdf). RINEX Observation
            Code must be paired with Satellite system to determine a unique identifier
    """

    signal_descriptor: TypeSatnavSignalDescriptorSignalDescriptor
