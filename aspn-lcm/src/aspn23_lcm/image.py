"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

"""
This code is generated via https://git.aspn.us/pntos/firehose/-/blob/main/firehose/backends/aspn/aspn_yaml_to_lcm.py
DO NOT hand edit code.  Make any changes required using the firehose repo instead
"""

from io import BytesIO
import struct

import aspn23_lcm

class image(object):
    """ 2D Raster Image """

    __slots__ = ["icd_image", "header", "time_of_validity", "height", "width", "is_bigendian", "image_type", "image_data_length", "image_data", "camera_model", "num_model_coefficients", "model_coefficients", "num_integrity", "integrity"]

    __typenames__ = ["int8_t", "aspn23_lcm.type_header", "aspn23_lcm.type_timestamp", "int64_t", "int64_t", "boolean", "int8_t", "int64_t", "int16_t", "int8_t", "int16_t", "double", "int16_t", "aspn23_lcm.type_integrity"]

    __dimensions__ = [None, None, None, None, None, None, None, None, ["image_data_length"], None, None, ["num_model_coefficients"], None, ["num_integrity"]]

    IMAGE_TYPE_BMP = 0
    """ Windows Bitmaps """
    IMAGE_TYPE_PNG = 1
    """ Portable Network Graphics """
    IMAGE_TYPE_JPG = 2
    """ Joint Photographic Experts Group """
    IMAGE_TYPE_TIFF = 3
    """ Tag Image File Format """
    IMAGE_TYPE_RAW_GRAY8 = 4
    """ Single channel raw gray scale image. One byte per pixel. """
    IMAGE_TYPE_RAW_RGB8 = 5
    """ Three channel raw RGB image. One byte per pixel per channel. """
    IMAGE_TYPE_RAW_BGR8 = 6
    """ Three channel raw BGR image. One byte per pixel per channel. """
    IMAGE_TYPE_RAW_RGBA8 = 7
    """ Four channel raw RGBA image. One byte per pixel per channel. """
    IMAGE_TYPE_RAW_BGRA8 = 8
    """ Four channel raw BGRA image. One byte per pixel per channel. """
    IMAGE_TYPE_RAW_GRAY16 = 9
    """ Single channel raw gray scale image. Two bytes per pixel. """
    IMAGE_TYPE_RAW_GRAYFLOAT64 = 10
    """ Single channel raw gray scale image. Eight bytes per pixel. """
    CAMERA_MODEL_ASPN_PINHOLE_PLUMB_BOB = 0
    """
    A 10 parameter model, 4 parameters composing the 2D focal length fc = (fx, fy) and camera
    principal point cc = (cx, cy) in pixels, a model of radial and tangential distortion specified using
    the 5 parameters (kc1, kc2, kc3, kc4, kc5). And the skew parameter, alpha_c. Mapping from 3D points
    in the world to 2D points in the image is described at
    http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
    """
    CAMERA_MODEL_ASPN_LINEAR_MODEL = 1
    """
    A 4 parameter model, modeling only the the 2D focal length fc = (fx, fy) and camera principal
    point cc = (cx, cy). Effectively using the same model as described in described at
    http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html, with all distortion and skew
    parameters zeroed out
    """

    def __init__(self):
        self.icd_image = 0
        """
        Non ASPN. Do not use. Extra field encoding the struct name to disambiguate LCM type fingerprint hashes.
        LCM Type: int8_t
        """

        self.header = aspn23_lcm.type_header()
        """
        Description: Standard ASPN measurement header.
        Units: none
        LCM Type: aspn23_lcm.type_header
        """

        self.time_of_validity = aspn23_lcm.type_timestamp()
        """
        Description: Time at which the measurement is considered to be valid.
        Units: none
        LCM Type: aspn23_lcm.type_timestamp
        """

        self.height = 0
        """
        Description: The number of pixel rows in this image.
        Units: pixels
        LCM Type: int64_t
        """

        self.width = 0
        """
        Description: The number of pixel columns in this image.
        Units: pixels
        LCM Type: int64_t
        """

        self.is_bigendian = False
        """
        Description: True if the image_data byte order is big endian, false if it is little endian. For
        raw image_types, this is only applicable for image_types with multi-byte pixels.
        Units: none
        LCM Type: boolean
        """

        self.image_type = 0
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
        LCM Type: int8_t
        """

        self.image_data_length = 0
        """
        Description: Length of the byte array holding the encoded image. For raw image types, this
        should be equal to height * width * num_channels * bytes_per_pixel.
        Units: byte
        LCM Type: int64_t
        """

        self.image_data = []
        """
        Description: Stores the encoded image. Interpretation varies based on the image_type.
        Units: none
        LCM Type: int16_t[image_data_length]
        """

        self.camera_model = 0
        """
        The model used to map 3D points in the world to 2D points on the image plane
        LCM Type: int8_t
        """

        self.num_model_coefficients = 0
        """
        Description: The number of coefficients used in camera model.
        Units: none
        LCM Type: int16_t
        """

        self.model_coefficients = []
        """
        Description: The camera model parameters, size depending on the distortion model.
        Units: none
        LCM Type: double[num_model_coefficients]
        """

        self.num_integrity = 0
        """
        Description: Number of integrity values.
        Units: none
        LCM Type: int16_t
        """

        self.integrity = []
        """
        Description: Measurement integrity. Includes the integrity method used and an integrity value
        (which is to be interpreted based upon the integrity method). The intent of allowing num_integrity >
        1 is to report multiple integrity values based on multiple integrity methods.
        Units: none
        LCM Type: aspn23_lcm.type_integrity[num_integrity]
        """


    def encode(self):
        buf = BytesIO()
        buf.write(image._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">b", self.icd_image))
        assert self.header._get_packed_fingerprint() == aspn23_lcm.type_header._get_packed_fingerprint()
        self.header._encode_one(buf)
        assert self.time_of_validity._get_packed_fingerprint() == aspn23_lcm.type_timestamp._get_packed_fingerprint()
        self.time_of_validity._encode_one(buf)
        buf.write(struct.pack(">qqbbq", self.height, self.width, self.is_bigendian, self.image_type, self.image_data_length))
        buf.write(struct.pack('>%dh' % self.image_data_length, *self.image_data[:self.image_data_length]))
        buf.write(struct.pack(">bh", self.camera_model, self.num_model_coefficients))
        buf.write(struct.pack('>%dd' % self.num_model_coefficients, *self.model_coefficients[:self.num_model_coefficients]))
        buf.write(struct.pack(">h", self.num_integrity))
        for i0 in range(self.num_integrity):
            assert self.integrity[i0]._get_packed_fingerprint() == aspn23_lcm.type_integrity._get_packed_fingerprint()
            self.integrity[i0]._encode_one(buf)

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != image._get_packed_fingerprint():
            raise ValueError("Decode error")
        return image._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = image()
        self.icd_image = struct.unpack(">b", buf.read(1))[0]
        self.header = aspn23_lcm.type_header._decode_one(buf)
        self.time_of_validity = aspn23_lcm.type_timestamp._decode_one(buf)
        self.height, self.width = struct.unpack(">qq", buf.read(16))
        self.is_bigendian = bool(struct.unpack('b', buf.read(1))[0])
        self.image_type, self.image_data_length = struct.unpack(">bq", buf.read(9))
        self.image_data = struct.unpack('>%dh' % self.image_data_length, buf.read(self.image_data_length * 2))
        self.camera_model, self.num_model_coefficients = struct.unpack(">bh", buf.read(3))
        self.model_coefficients = struct.unpack('>%dd' % self.num_model_coefficients, buf.read(self.num_model_coefficients * 8))
        self.num_integrity = struct.unpack(">h", buf.read(2))[0]
        self.integrity = []
        for i0 in range(self.num_integrity):
            self.integrity.append(aspn23_lcm.type_integrity._decode_one(buf))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if image in parents: return 0
        newparents = parents + [image]
        tmphash = (0xae29c48a79dfc920+ aspn23_lcm.type_header._get_hash_recursive(newparents)+ aspn23_lcm.type_timestamp._get_hash_recursive(newparents)+ aspn23_lcm.type_integrity._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if image._packed_fingerprint is None:
            image._packed_fingerprint = struct.pack(">Q", image._get_hash_recursive([]))
        return image._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", image._get_packed_fingerprint())[0]

