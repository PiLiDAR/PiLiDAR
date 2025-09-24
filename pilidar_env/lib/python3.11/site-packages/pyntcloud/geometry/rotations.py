import numpy as np


def Rx(angle, degrees=True):
    if degrees:
        cx = np.cos(np.deg2rad(angle))
        sx = np.sin(np.deg2rad(angle))
    else:
        cx = np.cos(angle)
        sx = np.sin(angle)
    return np.array([[1, 0, 0],
                     [0, cx, sx],
                     [0, -sx, cx]])


def Ry(angle, degrees=True):
    if degrees:
        cy = np.cos(np.deg2rad(angle))
        sy = np.sin(np.deg2rad(angle))
    else:
        cy = np.cos(angle)
        sy = np.sin(angle)
    return np.array([[cy, 0, -sy],
                     [0, 1, 0],
                     [sy, 0, cy]])


def Rz(angle, degrees=True):
    if degrees:
        cz = np.cos(np.deg2rad(angle))
        sz = np.sin(np.deg2rad(angle))
    else:
        cz = np.cos(angle)
        sz = np.sin(angle)
    return np.array([[cz, sz, 0],
                     [-sz, cz, 0],
                     [0, 0, 1]])
