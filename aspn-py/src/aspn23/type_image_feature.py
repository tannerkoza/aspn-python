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


@dataclass
class TypeImageFeature:
    """
    Image feature characteristics

    ### Attributes

    response - float:
            Value representing the magnitude of the response of the keypoint detection algorithm
            used to locate this keypoint in the image.

    orientation - float:
            Value representing the orientation of the detected keypoint, as measured relative to
            the image coordinate system (radians, clockwise). Metadata flag determines if this field
            is calculated by the current keypoint detector.

    size - float:
            Effective size (pixels) of the detected keypoint. This field takes into
            consideration the octave and kernel used to localize the keypoint, and normalizes to the
            original size of the image.

    class_id - int:
            ID that ties the detected keypoint to a class or group of keypoints.

    octave - int:
            Octave or pyramid layer from which the detected keypoint was extracted.

    descriptor - NumpyArray[int]:
            Feature descriptor.
    """

    response: float
    orientation: float
    size: float
    class_id: int
    octave: int
    descriptor: NumpyArray[int]
