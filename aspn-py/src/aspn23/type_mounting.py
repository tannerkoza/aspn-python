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
class TypeMounting:
    """
    Describes the translational and angular offset between the sensor frame and the platform body
    frame. Platform body frame is defined using axes in forward, right, and down convention, and the
    origin is system defined. This type also contains the uncertainty associated with the translational
    and angular offset.

    ### Attributes

    lever_arm - NumpyArray[float]:
            3x1 lever arm vector describing the sensor position in the platform body frame.

    lever_arm_sigma - NumpyArray[float]:
            3x1 lever arm uncertainty vector as standard deviations in the platform body frame.

    orientation_quaternion - Optional[Optional[NumpyArray[float]]]:
            Four element quaternion, q = [a, b, c, d], where a = cos(phi/2), b =
            (phi_x/phi)*sin(phi/2), c = (phi_y/phi)*sin(phi/2), and d = (phi_z/phi)*sin(phi/2). In
            this description, the vector [phi_x, phi_y, phi_z] represents the rotation vector that
            describes the frame rotation to be applied to the "reference" frame (ECI, ECEF, or NED)
            to rotate it into the axes that describe the measured attitude, and the value phi is the
            magnitude of the [phi_x, phi_y, phi_z] vector. Orientation is optional in the sense that
            orientation shall be specified except in the case that orientation is meaningless, for
            example, in the case of an RF antenna mounting. See "conventions" documentation for more
            detailed information.

    orientation_tilt_error_covariance - Optional[Optional[NumpyMatrix[float]]]:
            Tilt error covariance matrix. This matrix represents the uncertainty in the "tilt
            errors" that represent the additional rotation to be applied to the provided attitude
            quaternion in order to convert it to the true attitude with no errors. By convention,
            these "tilt errors" are expressed in the reference frame (ECI, ECEF, or NED). Tilt error
            covariance is optional only because orientation itself is optional. Orientation error
            covariance shall be provided if orientation is provided. See "conventions" documentation
            for more detailed information.
    """

    lever_arm: NumpyArray[float]
    lever_arm_sigma: NumpyArray[float]
    orientation_quaternion: Optional[NumpyArray[float]]
    orientation_tilt_error_covariance: Optional[NumpyMatrix[float]]
