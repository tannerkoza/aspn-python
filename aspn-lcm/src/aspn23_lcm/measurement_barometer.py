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

class measurement_barometer(object):
    """ Barometric pressure. """

    __slots__ = ["icd_measurement_barometer", "header", "time_of_validity", "pressure", "variance", "error_model", "num_error_model_params", "error_model_params", "num_integrity", "integrity"]

    __typenames__ = ["int8_t", "aspn23_lcm.type_header", "aspn23_lcm.type_timestamp", "double", "double", "int8_t", "int32_t", "double", "int16_t", "aspn23_lcm.type_integrity"]

    __dimensions__ = [None, None, None, None, None, None, None, ["num_error_model_params"], None, ["num_integrity"]]

    ERROR_MODEL_NONE = 0
    """ No additional error model provided (num_error_model_params = 0). """

    def __init__(self):
        self.icd_measurement_barometer = 0
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

        self.pressure = 0.0
        """
        Description: Reading from a barometric pressure sensor.
        Units: pascal
        LCM Type: double
        """

        self.variance = 0.0
        """
        Description: Pressure variance.
        Units: pascal^2
        LCM Type: double
        """

        self.error_model = 0
        """
        Defines an optional error model for other than zero-mean, additive, white Gaussian noise (AWGN).
        LCM Type: int8_t
        """

        self.num_error_model_params = 0
        """
        Description: Number of parameters required for the error model chosen.
        Units: none
        LCM Type: int32_t
        """

        self.error_model_params = []
        """
        Description: Error model parameters that characterize the optional error model.
        Units: various
        LCM Type: double[num_error_model_params]
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
        buf.write(measurement_barometer._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">b", self.icd_measurement_barometer))
        assert self.header._get_packed_fingerprint() == aspn23_lcm.type_header._get_packed_fingerprint()
        self.header._encode_one(buf)
        assert self.time_of_validity._get_packed_fingerprint() == aspn23_lcm.type_timestamp._get_packed_fingerprint()
        self.time_of_validity._encode_one(buf)
        buf.write(struct.pack(">ddbi", self.pressure, self.variance, self.error_model, self.num_error_model_params))
        buf.write(struct.pack('>%dd' % self.num_error_model_params, *self.error_model_params[:self.num_error_model_params]))
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
        if buf.read(8) != measurement_barometer._get_packed_fingerprint():
            raise ValueError("Decode error")
        return measurement_barometer._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = measurement_barometer()
        self.icd_measurement_barometer = struct.unpack(">b", buf.read(1))[0]
        self.header = aspn23_lcm.type_header._decode_one(buf)
        self.time_of_validity = aspn23_lcm.type_timestamp._decode_one(buf)
        self.pressure, self.variance, self.error_model, self.num_error_model_params = struct.unpack(">ddbi", buf.read(21))
        self.error_model_params = struct.unpack('>%dd' % self.num_error_model_params, buf.read(self.num_error_model_params * 8))
        self.num_integrity = struct.unpack(">h", buf.read(2))[0]
        self.integrity = []
        for i0 in range(self.num_integrity):
            self.integrity.append(aspn23_lcm.type_integrity._decode_one(buf))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if measurement_barometer in parents: return 0
        newparents = parents + [measurement_barometer]
        tmphash = (0xcfac5ae24fc0e0c4+ aspn23_lcm.type_header._get_hash_recursive(newparents)+ aspn23_lcm.type_timestamp._get_hash_recursive(newparents)+ aspn23_lcm.type_integrity._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if measurement_barometer._packed_fingerprint is None:
            measurement_barometer._packed_fingerprint = struct.pack(">Q", measurement_barometer._get_hash_recursive([]))
        return measurement_barometer._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", measurement_barometer._get_packed_fingerprint())[0]

