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

class type_mounting(object):
    """
    Describes the translational and angular offset between the sensor frame and the platform body frame.
    Platform body frame is defined using axes in forward, right, and down convention, and the origin is
    system defined. This type also contains the uncertainty associated with the translational and
    angular offset.
    """

    __slots__ = ["icd_type_mounting", "lever_arm", "lever_arm_sigma", "orientation_quaternion", "orientation_tilt_error_covariance"]

    __typenames__ = ["int8_t", "double", "double", "double", "double"]

    __dimensions__ = [None, [3], [3], [4], [3, 3]]

    def __init__(self):
        self.icd_type_mounting = 0
        """
        Non ASPN. Do not use. Extra field encoding the struct name to disambiguate LCM type fingerprint hashes.
        LCM Type: int8_t
        """

        self.lever_arm = [ 0.0 for dim0 in range(3) ]
        """
        Description: 3x1 lever arm vector describing the sensor position in the platform body frame.
        Units: m
        LCM Type: double[3]
        """

        self.lever_arm_sigma = [ 0.0 for dim0 in range(3) ]
        """
        Description: 3x1 lever arm uncertainty vector as standard deviations in the platform body frame.
        Units: m
        LCM Type: double[3]
        """

        self.orientation_quaternion = [ 0.0 for dim0 in range(4) ]
        """
        Description: Four element quaternion, q = [a, b, c, d], where a = cos(phi/2), b =
        (phi_x/phi)*sin(phi/2), c = (phi_y/phi)*sin(phi/2), and d = (phi_z/phi)*sin(phi/2). In this
        description, the vector [phi_x, phi_y, phi_z] represents the rotation vector that describes the
        frame rotation to be applied to the "reference" frame (ECI, ECEF, or NED) to rotate it into the axes
        that describe the measured attitude, and the value phi is the magnitude of the [phi_x, phi_y, phi_z]
        vector.
        Orientation is optional in the sense that orientation shall be specified except in the case that
        orientation is meaningless, for example, in the case of an RF antenna mounting.
        See "conventions" documentation for more detailed information.
        Units: none
        LCM Type: double[4]
        """

        self.orientation_tilt_error_covariance = [ [ 0.0 for dim1 in range(3) ] for dim0 in range(3) ]
        """
        Description: Tilt error covariance matrix. This matrix represents the uncertainty in the "tilt
        errors" that represent the additional rotation to be applied to the provided attitude quaternion in
        order to convert it to the true attitude with no errors. By convention, these "tilt errors" are
        expressed in the reference frame (ECI, ECEF, or NED).
        Tilt error covariance is optional only because orientation itself is optional. Orientation error
        covariance shall be provided if orientation is provided.
        See "conventions" documentation for more detailed information.
        Units: rad^2
        LCM Type: double[3][3]
        """


    def encode(self):
        buf = BytesIO()
        buf.write(type_mounting._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">b", self.icd_type_mounting))
        buf.write(struct.pack('>3d', *self.lever_arm[:3]))
        buf.write(struct.pack('>3d', *self.lever_arm_sigma[:3]))
        buf.write(struct.pack('>4d', *self.orientation_quaternion[:4]))
        for i0 in range(3):
            buf.write(struct.pack('>3d', *self.orientation_tilt_error_covariance[i0][:3]))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != type_mounting._get_packed_fingerprint():
            raise ValueError("Decode error")
        return type_mounting._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = type_mounting()
        self.icd_type_mounting = struct.unpack(">b", buf.read(1))[0]
        self.lever_arm = struct.unpack('>3d', buf.read(24))
        self.lever_arm_sigma = struct.unpack('>3d', buf.read(24))
        self.orientation_quaternion = struct.unpack('>4d', buf.read(32))
        self.orientation_tilt_error_covariance = []
        for i0 in range(3):
            self.orientation_tilt_error_covariance.append(struct.unpack('>3d', buf.read(24)))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if type_mounting in parents: return 0
        tmphash = (0xd0f86db27c5effbb) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if type_mounting._packed_fingerprint is None:
            type_mounting._packed_fingerprint = struct.pack(">Q", type_mounting._get_hash_recursive([]))
        return type_mounting._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", type_mounting._get_packed_fingerprint())[0]

