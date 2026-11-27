"""The arithmetic of the dead-reckoning preprocessor, with no ROS imports.

Separate from ``preprocess.py`` so it can be tested on any machine (``tests/test_dvl_dr.py``) rather
than only where ROS 2 is installed. The same split ``ros/src/sonar_relay`` makes with its pure ``projected``,
taken one step further because none of this needs a message type at all.
"""
from __future__ import annotations

G = 9.80665
#: the covariance the relay stamps on a report with no bottom lock; at or above it, there is no fix
NO_FIX = 1e5


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def correct_lever_arm(v, omega, r):
    """``v_body = v_dvl - omega x r``. All three in the body's FLU axes, ``r`` from the body origin to
    the acoustic centre.

    A DVL measures the velocity *of the DVL*. With the instrument 12 cm under the hull, a 0.4 rad/s
    pitch rate is already 5 cm/s of phantom surge, and a filter with no mount model integrates it
    straight into the track.
    """
    w = cross(omega, r)
    return (v[0] - w[0], v[1] - w[1], v[2] - w[2])


def has_fix(covariance) -> bool:
    """True when a twist's linear block is an actual measurement rather than a lost bottom lock.

    ``covariance`` is the 36-element row-major 6x6 of ``TwistWithCovarianceStamped``.
    """
    return all(0.0 < covariance[i] < NO_FIX for i in (0, 7, 14))


def depth_pose(pressure_pa: float, surface_pa: float, rho: float, water_level: float):
    """Hydrostatic depth as ``(z in the tank's ENU frame, depth below the surface)``.

    ``z`` is measured up from the tank floor, so it is ``water_level`` minus the depth - the same
    relation ``vehicles.common.frames.position_tank_to_ned`` uses at the other seam. The depth is clamped at
    zero: a pressure below the surface value means the baro is off, not that the vehicle is airborne.
    """
    depth = max((float(pressure_pa) - float(surface_pa)) / (float(rho) * G), 0.0)
    return float(water_level) - depth, depth
