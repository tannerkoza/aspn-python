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

class type_satnav_time(object):
    """ Satellite system time """

    __slots__ = ["icd_type_satnav_time", "week_number", "seconds_of_week", "time_reference"]

    __typenames__ = ["int8_t", "int32_t", "double", "int8_t"]

    __dimensions__ = [None, None, None, None]

    TIME_REFERENCE_TIME_GPS = 0
    """ GPS system time. """
    TIME_REFERENCE_TIME_GALILEO = 1
    """ Galileo system time. """
    TIME_REFERENCE_TIME_BEIDOU = 2
    """ BeiDou system time. """
    TIME_REFERENCE_TIME_GLONASS = 3
    """ GLONASS system time. """

    def __init__(self):
        self.icd_type_satnav_time = 0
        """
        Non ASPN. Do not use. Extra field encoding the struct name to disambiguate LCM type fingerprint hashes.
        LCM Type: int8_t
        """

        self.week_number = 0
        """
        Description: Full Week number since zero epoch at which the data provided in this message is
        valid, expressed in time system defined by time_reference enum (below).
        Units: weeks
        LCM Type: int32_t
        """

        self.seconds_of_week = 0.0
        """
        Description: Seconds since start of current week at which the data provided in this message is
        valid, expressed in time system defined by time_reference enum (below).
        Units: seconds
        LCM Type: double
        """

        self.time_reference = 0
        """
        Reference time system used to express the data in this message. In a multi-GNSS receiver
        (GPS/GLONASS/Galileo/QZSS/BeiDou) all pseudorange observations must refer to one receiver clock
        only. The receiver clock time of the measurement is the receiver clock time of the received signals.
        It is identical for the phase and range measurements and is identical for all satellites observed in
        a given epoch.
        LCM Type: int8_t
        """


    def encode(self):
        buf = BytesIO()
        buf.write(type_satnav_time._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">bidb", self.icd_type_satnav_time, self.week_number, self.seconds_of_week, self.time_reference))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != type_satnav_time._get_packed_fingerprint():
            raise ValueError("Decode error")
        return type_satnav_time._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = type_satnav_time()
        self.icd_type_satnav_time, self.week_number, self.seconds_of_week, self.time_reference = struct.unpack(">bidb", buf.read(14))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if type_satnav_time in parents: return 0
        tmphash = (0x77ce62b549494fa1) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if type_satnav_time._packed_fingerprint is None:
            type_satnav_time._packed_fingerprint = struct.pack(">Q", type_satnav_time._get_hash_recursive([]))
        return type_satnav_time._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", type_satnav_time._get_packed_fingerprint())[0]

