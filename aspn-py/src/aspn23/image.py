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


class ImageImageType(Enum):
    """
    Enumerated field which specifies the datatype of the pixels represented in this image.
    Raw images are stored in row-major order. In multi-channel raw images, the first element
    contains the first pixel of the first channel, the second element contains the first pixel of the
    second channel, and so on. The data of each multi-byte pixel should be contiguous within the data
    array with the endianness defined by the is_bigendian field.
    Example - 2x2 RAW_RGB8 image_data[0] row1 col1 chanR image_data[1] row1 col1 chanG image_data[2]
    row1 col1 chanB image_data[3] row1 col2 chanR image_data[4] row1 col2 chanG image_data[5] row1 col2
    chanB image_data[6] row2 col1 chanR image_data[7] row2 col1 chanG image_data[8] row2 col1 chanB
    image_data[9] row2 col2 chanR image_data[10] row2 col2 chanG image_data[11] row2 col2 chanB
    Example - 2x2 RAW_GRAY16, is_bigendian = 1 Where byte 1 is the most significant byte (byte1 << 8
    | byte2): image_data[0] row1 col1 byte1 image_data[1] row1 col1 byte2 image_data[2] row1 col2 byte1
    image_data[3] row1 col2 byte2 image_data[4] row2 col1 byte1 image_data[5] row2 col1 byte2
    image_data[6] row2 col2 byte1 image_data[7] row2 col2 byte2
    """

    """
    Windows Bitmaps
    """
    BMP = 0

    """
    Portable Network Graphics
    """
    PNG = 1

    """
    Joint Photographic Experts Group
    """
    JPG = 2

    """
    Tag Image File Format
    """
    TIFF = 3

    """
    Single channel raw gray scale image. One byte per pixel.
    """
    RAW_GRAY8 = 4

    """
    Three channel raw RGB image. One byte per pixel per channel.
    """
    RAW_RGB8 = 5

    """
    Three channel raw BGR image. One byte per pixel per channel.
    """
    RAW_BGR8 = 6

    """
    Four channel raw RGBA image. One byte per pixel per channel.
    """
    RAW_RGBA8 = 7

    """
    Four channel raw BGRA image. One byte per pixel per channel.
    """
    RAW_BGRA8 = 8

    """
    Single channel raw gray scale image. Two bytes per pixel.
    """
    RAW_GRAY16 = 9

    """
    Single channel raw gray scale image. Eight bytes per pixel.
    """
    RAW_GRAYFLOAT64 = 10


class ImageCameraModel(Enum):
    """
    The model used to map 3D points in the world to 2D points on the image plane
    """

    """
    A 10 parameter model, 4 parameters composing the 2D focal length fc = (fx, fy) and camera
    principal point cc = (cx, cy) in pixels, a model of radial and tangential distortion specified using
    the 5 parameters (kc1, kc2, kc3, kc4, kc5). And the skew parameter, alpha_c. Mapping from 3D points
    in the world to 2D points in the image is described at
    http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
    """
    ASPN_PINHOLE_PLUMB_BOB = 0

    """
    A 4 parameter model, modeling only the the 2D focal length fc = (fx, fy) and camera principal
    point cc = (cx, cy). Effectively using the same model as described in described at
    http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html, with all distortion and skew
    parameters zeroed out
    """
    ASPN_LINEAR_MODEL = 1


@dataclass
class Image(AspnBase):
    """
    2D Raster Image

    ### Attributes

    header - TypeHeader:
            Standard ASPN measurement header.

    time_of_validity - TypeTimestamp:
            Time at which the measurement is considered to be valid.

    height - int:
            The number of pixel rows in this image.

    width - int:
            The number of pixel columns in this image.

    is_bigendian - bool:
            True if the image_data byte order is big endian, false if it is little endian. For
            raw image_types, this is only applicable for image_types with multi-byte pixels.

    image_type - ImageImageType:
            Enumerated field which specifies the datatype of the pixels represented in this
            image. Raw images are stored in row-major order. In multi-channel raw images, the first
            element contains the first pixel of the first channel, the second element contains the
            first pixel of the second channel, and so on. The data of each multi-byte pixel should
            be contiguous within the data array with the endianness defined by the is_bigendian
            field. Example - 2x2 RAW_RGB8 image_data[0] row1 col1 chanR image_data[1] row1 col1
            chanG image_data[2] row1 col1 chanB image_data[3] row1 col2 chanR image_data[4] row1
            col2 chanG image_data[5] row1 col2 chanB image_data[6] row2 col1 chanR image_data[7]
            row2 col1 chanG image_data[8] row2 col1 chanB image_data[9] row2 col2 chanR
            image_data[10] row2 col2 chanG image_data[11] row2 col2 chanB Example - 2x2 RAW_GRAY16,
            is_bigendian = 1 Where byte 1 is the most significant byte (byte1 << 8 | byte2):
            image_data[0] row1 col1 byte1 image_data[1] row1 col1 byte2 image_data[2] row1 col2
            byte1 image_data[3] row1 col2 byte2 image_data[4] row2 col1 byte1 image_data[5] row2
            col1 byte2 image_data[6] row2 col2 byte1 image_data[7] row2 col2 byte2

    image_data - NumpyArray[int]:
            Stores the encoded image. Interpretation varies based on the image_type.

    camera_model - ImageCameraModel:
            The model used to map 3D points in the world to 2D points on the image plane

    model_coefficients - NumpyArray[float]:
            The camera model parameters, size depending on the distortion model.

    integrity - List[TypeIntegrity]:
            Measurement integrity. Includes the integrity method used and an integrity value
            (which is to be interpreted based upon the integrity method). The intent of allowing
            num_integrity > 1 is to report multiple integrity values based on multiple integrity
            methods.
    """

    header: TypeHeader
    time_of_validity: TypeTimestamp
    height: int
    width: int
    is_bigendian: bool
    image_type: ImageImageType
    image_data: NumpyArray[int]
    camera_model: ImageCameraModel
    model_coefficients: NumpyArray[float]
    integrity: List[TypeIntegrity]
