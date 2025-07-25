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

class metadata_GPS_Lnav_ephemeris(object):
    """
    LNAV Ephemeris describing GPS satellite locations. Definitions and usage are covered in
    ICD-GPS-200L, Section 20.3.3.4 and following.
    """

    __slots__ = ["icd_metadata_GPS_Lnav_ephemeris", "info", "time_of_validity", "week_number", "prn", "clock", "orbit", "t_gd", "iodc", "iode"]

    __typenames__ = ["int8_t", "aspn23_lcm.type_metadataheader", "aspn23_lcm.type_timestamp", "int32_t", "int32_t", "aspn23_lcm.type_satnav_clock", "aspn23_lcm.type_kepler_orbit", "double", "int32_t", "int16_t"]

    __dimensions__ = [None, None, None, None, None, None, None, None, None, None]

    def __init__(self):
        self.icd_metadata_GPS_Lnav_ephemeris = 0
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

        self.week_number = 0
        """
        Description: Full GPS week number calculated from the Modulo 1024 WN in Subframe 1 and the
        number of GPS week rollovers
        Units: weeks
        LCM Type: int32_t
        """

        self.prn = 0
        """
        Description: Satellite PRN number.
        Units: none
        LCM Type: int32_t
        """

        self.clock = aspn23_lcm.type_satnav_clock()
        """
        Description: GNSS broadcast parameters required to calculate sv clock corrections.
        Units: none
        LCM Type: aspn23_lcm.type_satnav_clock
        """

        self.orbit = aspn23_lcm.type_kepler_orbit()
        """
        Description: Keplerian orbit parameters required to calculate satellite position.
        Units: none
        LCM Type: aspn23_lcm.type_kepler_orbit
        """

        self.t_gd = 0.0
        """
        Description: Group delay differential between L1 and L2.
        Units: s
        LCM Type: double
        """

        self.iodc = 0
        """
        Description: Issue of Data Clock. 10 bit value from Subframe 1
        Units: none
        LCM Type: int32_t
        """

        self.iode = 0
        """
        Description: Issue of Data Ephemeris. 8 bits repeated in Subframe 2 and Subframe 3. Should match
        the 8 LSBs of the IODC.
        Units: none
        LCM Type: int16_t
        """


    def encode(self):
        buf = BytesIO()
        buf.write(metadata_GPS_Lnav_ephemeris._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">b", self.icd_metadata_GPS_Lnav_ephemeris))
        assert self.info._get_packed_fingerprint() == aspn23_lcm.type_metadataheader._get_packed_fingerprint()
        self.info._encode_one(buf)
        assert self.time_of_validity._get_packed_fingerprint() == aspn23_lcm.type_timestamp._get_packed_fingerprint()
        self.time_of_validity._encode_one(buf)
        buf.write(struct.pack(">ii", self.week_number, self.prn))
        assert self.clock._get_packed_fingerprint() == aspn23_lcm.type_satnav_clock._get_packed_fingerprint()
        self.clock._encode_one(buf)
        assert self.orbit._get_packed_fingerprint() == aspn23_lcm.type_kepler_orbit._get_packed_fingerprint()
        self.orbit._encode_one(buf)
        buf.write(struct.pack(">dih", self.t_gd, self.iodc, self.iode))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != metadata_GPS_Lnav_ephemeris._get_packed_fingerprint():
            raise ValueError("Decode error")
        return metadata_GPS_Lnav_ephemeris._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = metadata_GPS_Lnav_ephemeris()
        self.icd_metadata_GPS_Lnav_ephemeris = struct.unpack(">b", buf.read(1))[0]
        self.info = aspn23_lcm.type_metadataheader._decode_one(buf)
        self.time_of_validity = aspn23_lcm.type_timestamp._decode_one(buf)
        self.week_number, self.prn = struct.unpack(">ii", buf.read(8))
        self.clock = aspn23_lcm.type_satnav_clock._decode_one(buf)
        self.orbit = aspn23_lcm.type_kepler_orbit._decode_one(buf)
        self.t_gd, self.iodc, self.iode = struct.unpack(">dih", buf.read(14))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if metadata_GPS_Lnav_ephemeris in parents: return 0
        newparents = parents + [metadata_GPS_Lnav_ephemeris]
        tmphash = (0x64b0d0ca394139a4+ aspn23_lcm.type_metadataheader._get_hash_recursive(newparents)+ aspn23_lcm.type_timestamp._get_hash_recursive(newparents)+ aspn23_lcm.type_satnav_clock._get_hash_recursive(newparents)+ aspn23_lcm.type_kepler_orbit._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if metadata_GPS_Lnav_ephemeris._packed_fingerprint is None:
            metadata_GPS_Lnav_ephemeris._packed_fingerprint = struct.pack(">Q", metadata_GPS_Lnav_ephemeris._get_hash_recursive([]))
        return metadata_GPS_Lnav_ephemeris._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", metadata_GPS_Lnav_ephemeris._get_packed_fingerprint())[0]

