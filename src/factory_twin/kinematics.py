"""Forward kinematics for the 3-DOF arm — the bridge that lets a GPU-free
viewer show the arm's *driven* pose.

A UsdPhysics articulation stores only the rest pose in its link transforms; the
motion lives in the revolute joints' drive targets and is realised by a solver
(Isaac Sim). To visualise that motion without a solver we recompute each link's
position analytically from the joint angles — plain forward kinematics.

The chain and segment lengths mirror physics.build_robot_arm_articulated():

    base_top (yaw joint, z=0.40)
       │ 0.20        (turntable stub)
    shoulder (pitch joint, z=0.60)
       │ 1.00        (upper arm, link1)
    elbow (pitch joint, z=1.60)
       │ 0.77        (forearm + gripper, link2)
    tip
"""

from __future__ import annotations

import math

import numpy as np

# segment lengths (metres), see module docstring
_L_STUB, _L_UPPER, _L_FORE = 0.20, 1.00, 0.77
_BASE_TOP = np.array([0.0, 0.0, 0.40])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def arm_fk(q0_deg: float, q1_deg: float, q2_deg: float):
    """Return the arm's key joint points in the robot's LOCAL frame:
    [base_top, shoulder, elbow, tip].

    q0 = yaw about Z, q1 = shoulder pitch about Y, q2 = elbow pitch about Y —
    the same axes and sign convention as the UsdPhysics revolute joints, so the
    drawn pose matches what PhysX will produce from the same drive targets.
    """
    q0, q1, q2 = map(math.radians, (q0_deg, q1_deg, q2_deg))
    r0 = _rz(q0)
    shoulder = _BASE_TOP + r0 @ np.array([0, 0, _L_STUB])
    r1 = r0 @ _ry(q1)
    elbow = shoulder + r1 @ np.array([0, 0, _L_UPPER])
    r2 = r1 @ _ry(q2)
    tip = elbow + r2 @ np.array([0, 0, _L_FORE])
    return [_BASE_TOP, shoulder, elbow, tip]
