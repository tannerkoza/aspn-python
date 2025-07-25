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

class metadata_GLONASS_ephemeris(object):
    """
    GLONASS Ephemeris describing GLONASS satellite locations. Definitions and usage are covered in
    GLONASS ICD L1,L2 - Edition 5.1 2008, Section 4.4.
    """

    __slots__ = ["icd_metadata_GLONASS_ephemeris", "info", "time_of_validity", "slot_number", "freq_o", "n_t", "t_k", "t_b", "gamma_n", "tau_n", "sv_pos_x", "sv_vel_x", "sv_accel_x", "sv_pos_y", "sv_vel_y", "sv_accel_y", "sv_pos_z", "sv_vel_z", "sv_accel_z"]

    __typenames__ = ["int8_t", "aspn23_lcm.type_metadataheader", "aspn23_lcm.type_timestamp", "int32_t", "int32_t", "int32_t", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"]

    __dimensions__ = [None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None]

    def __init__(self):
        self.icd_metadata_GLONASS_ephemeris = 0
        """
        Non ASPN. Do not use. Extra field encoding the struct name to disambiguate LCM type fingerprint hashes.
        LCM Type: int8_t
        """

        self.info = aspn23_lcm.type_metadataheader()
        """
        Description: Standard ASPN metadata header.
        Units: none
        LCM Type: aspn23_lcm.type_metadataheader
        """

        self.time_of_validity = aspn23_lcm.type_timestamp()
        """
        Description: Time at which the measurement is considered to be valid.
        Units: none
        LCM Type: aspn23_lcm.type_timestamp
        """

        self.slot_number = 0
        """
        Description: GLONASS satellite slot number.
        Units: none
        LCM Type: int32_t
        """

        self.freq_o = 0
        """
        Description: Frequency channel offset number in range from 0 to 20.
        Units: none
        LCM Type: int32_t
        """

        self.n_t = 0
        """
        Description: Calender number of day within 4 year interval starting at Jan 1 of a leap year
        Units: days
        LCM Type: int32_t
        """

        self.t_k = 0.0
        """
        Description: Time referenced to the beginning of the frame within the current day.
        Units: s
        LCM Type: double
        """

        self.t_b = 0.0
        """
        Description: Index of time interval within current day according to UTC(SU) + 03 hrs.
        Units: minutes
        LCM Type: double
        """

        self.gamma_n = 0.0
        """
        Description: Relative Satellite frequency bias
        Units: s/s
        LCM Type: double
        """

        self.tau_n = 0.0
        """
        Description: Satellite clock bias.
        Units: s
        LCM Type: double
        """

        self.sv_pos_x = 0.0
        """
        Description: Satellite X position in PZ-90 coordinate system at time t_b.
        Units: km
        LCM Type: double
        """

        self.sv_vel_x = 0.0
        """
        Description: Satellite X velocity in PZ-90 coordinate system at time t_b.
        Units: km/s
        LCM Type: double
        """

        self.sv_accel_x = 0.0
        """
        Description: Satellite X acceleration in PZ-90 coordinate system at time t_b.
        Units: km/s^2
        LCM Type: double
        """

        self.sv_pos_y = 0.0
        """
        Description: Satellite Y position in PZ-90 coordinate system at time t_b.
        Units: km
        LCM Type: double
        """

        self.sv_vel_y = 0.0
        """
        Description: Satellite Y velocity in PZ-90 coordinate system at time t_b.
        Units: km/s
        LCM Type: double
        """

        self.sv_accel_y = 0.0
        """
        Description: Satellite Y acceleration in PZ-90 coordinate system at time t_b.
        Units: km/s^2
        LCM Type: double
        """

        self.sv_pos_z = 0.0
        """
        Description: Satellite Z position in PZ-90 coordinate system at time t_b.
        Units: km
        LCM Type: double
        """

        self.sv_vel_z = 0.0
        """
        Description: Satellite Z velocity in PZ-90 coordinate system at time t_b.
        Units: km/s
        LCM Type: double
        """

        self.sv_accel_z = 0.0
        """
        Description: Satellite Z acceleration in PZ-90 coordinate system at time t_b.
        Units: km/s^2
        LCM Type: double
        """


    def encode(self):
        buf = BytesIO()
        buf.write(metadata_GLONASS_ephemeris._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">b", self.icd_metadata_GLONASS_ephemeris))
        assert self.info._get_packed_fingerprint() == aspn23_lcm.type_metadataheader._get_packed_fingerprint()
        self.info._encode_one(buf)
        assert self.time_of_validity._get_packed_fingerprint() == aspn23_lcm.type_timestamp._get_packed_fingerprint()
        self.time_of_validity._encode_one(buf)
        buf.write(struct.pack(">iiiddddddddddddd", self.slot_number, self.freq_o, self.n_t, self.t_k, self.t_b, self.gamma_n, self.tau_n, self.sv_pos_x, self.sv_vel_x, self.sv_accel_x, self.sv_pos_y, self.sv_vel_y, self.sv_accel_y, self.sv_pos_z, self.sv_vel_z, self.sv_accel_z))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != metadata_GLONASS_ephemeris._get_packed_fingerprint():
            raise ValueError("Decode error")
        return metadata_GLONASS_ephemeris._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = metadata_GLONASS_ephemeris()
        self.icd_metadata_GLONASS_ephemeris = struct.unpack(">b", buf.read(1))[0]
        self.info = aspn23_lcm.type_metadataheader._decode_one(buf)
        self.time_of_validity = aspn23_lcm.type_timestamp._decode_one(buf)
        self.slot_number, self.freq_o, self.n_t, self.t_k, self.t_b, self.gamma_n, self.tau_n, self.sv_pos_x, self.sv_vel_x, self.sv_accel_x, self.sv_pos_y, self.sv_vel_y, self.sv_accel_y, self.sv_pos_z, self.sv_vel_z, self.sv_accel_z = struct.unpack(">iiiddddddddddddd", buf.read(116))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if metadata_GLONASS_ephemeris in parents: return 0
        newparents = parents + [metadata_GLONASS_ephemeris]
        tmphash = (0x85ce7454a3af5357+ aspn23_lcm.type_metadataheader._get_hash_recursive(newparents)+ aspn23_lcm.type_timestamp._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if metadata_GLONASS_ephemeris._packed_fingerprint is None:
            metadata_GLONASS_ephemeris._packed_fingerprint = struct.pack(">Q", metadata_GLONASS_ephemeris._get_hash_recursive([]))
        return metadata_GLONASS_ephemeris._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", metadata_GLONASS_ephemeris._get_packed_fingerprint())[0]

