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
from .type_timestamp import TypeTimestamp


class MetadataImageFeaturesKeypointDetector(Enum):
    """
    Enumerated field which provides the type of keypoint detector used.
    """

    """
    Keypoint detected using AGAST based detector.
    """
    DET_AGAST = 0

    """
    Keypoint detected using AKAZE based detector.
    """
    DET_AKAZE = 1

    """
    Keypoint detected using BRISK based detector.
    """
    DET_BRISK = 2

    """
    Keypoint detected using FAST based detector.
    """
    DET_FAST = 3

    """
    Keypoint detected using GFTT based detector.
    """
    DET_GFTT = 4

    """
    Keypoint detected using KAZE based detector.
    """
    DET_KAZE = 5

    """
    Keypoint detected using MSER based detector.
    """
    DET_MSER = 6

    """
    Keypoint detected using ORB based detector.
    """
    DET_ORB = 7

    """
    Keypoint detected using SIFT based detector.
    """
    DET_SIFT = 8

    """
    Keypoint detected using SURF based detector.
    """
    DET_SURF = 9

    """
    Keypoint detected using Harris based detector.
    """
    DET_HARRIS = 10

    """
    Keypoint detected using Shi-Tomasi based detector.
    """
    DET_SHI = 11

    """
    Keypoint detected using a detector not represented in this enum.
    """
    DET_OTHER = 12


class MetadataImageFeaturesDescriptorExtractor(Enum):
    """
    Enumerated field which provides the type of descriptor extractor used to generate the
    measurement descriptor vector.
    """

    """
    Descriptor computed using the AKAZE extractor.
    """
    DESC_AKAZE = 0

    """
    Descriptor computed using the BRISK extractor.
    """
    DESC_BRISK = 1

    """
    Descriptor computed using the KAZE extractor.
    """
    DESC_KAZE = 2

    """
    Descriptor computed using the ORB extractor.
    """
    DESC_ORB = 3

    """
    Descriptor computed using the SIFT extractor.
    """
    DESC_SIFT = 4

    """
    Descriptor computed using the SURF extractor.
    """
    DESC_SURF = 5

    """
    Descriptor computed using an extractor not represented in this enum.
    """
    DESC_OTHER = 6


class MetadataImageFeaturesDescriptorType(Enum):
    """
    Enumerated field describing the underlying datatype of the provided descriptor vector.
    """

    """
    The descriptor data should be interpreted as an array of unsigned 8-bit ints.
    """
    UINT8 = 0

    """
    The descriptor data should be interpreted as an array of 32-bit floats.
    """
    FLOAT32 = 1


@dataclass
class MetadataImageFeatures(AspnBase):
    """
    Features from an optical camera.

    ### Attributes

    info - TypeMetadataheader:
            Standard ASPN metadata header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    keypoint_detector - MetadataImageFeaturesKeypointDetector:
            Enumerated field which provides the type of keypoint detector used.

    orientation_calculated - bool:
            Set to true if the keypoint detector or descriptor extractor calculates and assigns
            a primary orientation to the features represented in the measurements.

    descriptor_extractor - MetadataImageFeaturesDescriptorExtractor:
            Enumerated field which provides the type of descriptor extractor used to generate
            the measurement descriptor vector.

    is_bigendian - bool:
            True if the descriptor byte order is big endian, false if it is little endian. Only
            applicable for descriptor_types with multi-byte elements.

    descriptor_type - MetadataImageFeaturesDescriptorType:
            Enumerated field describing the underlying datatype of the provided descriptor
            vector.

    descriptor_number_of_elements - int:
            The number of elements of type 'descriptor_type' in the descriptor data array. In
            type_image_feature, descriptor data is represented as an array of uint8 values whose
            length is given by 'descriptor_size'. Inconjunction with descriptor_type, these fields
            describe how that data should be interpreted. For example - descriptor_type UINT8:
            descriptor_number_of_elements == descriptor_size - descriptor_type FLOAT32:
            (descriptor_number_of_elements * 4) == descriptor_size
    """

    info: TypeMetadataheader
    time_of_validity: TypeTimestamp
    keypoint_detector: MetadataImageFeaturesKeypointDetector
    orientation_calculated: bool
    descriptor_extractor: MetadataImageFeaturesDescriptorExtractor
    is_bigendian: bool
    descriptor_type: MetadataImageFeaturesDescriptorType
    descriptor_number_of_elements: int
