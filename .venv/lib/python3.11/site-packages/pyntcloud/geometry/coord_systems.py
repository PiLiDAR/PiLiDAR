import numpy as np


def spherical_to_cartesian(r, theta, phi, degrees=True):
    """
    Convert spherical coordinates (r, theta, phi) to cartesian (x, y, z).

    Parameters
    ----------
    r: (N,) ndarray
        Radial distance.
    theta: (N,) ndarray
        Azimuthal angle.
    phi: (N,) ndarray
        Polar angle.
    degrees: bool, optional
        If True, theta and phi will be assumed to be in degrees.

    Returns
    -------
    xyz: (N, 3) ndarray
        Corresponding cartesian coordinates.

    Notes
    -----
    Use notation of mathematical systems, NOT physics.
    """
    if degrees:
        theta = np.deg2rad(theta)
        phi = np.deg2rad(phi)

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)

    xyz = np.empty((r.shape[0], 3), dtype=np.float32)

    xyz[:, 0] = r * sin_phi * cos_theta
    xyz[:, 1] = r * sin_phi * sin_theta
    xyz[:, 2] = r * cos_phi

    return xyz


def cartesian_to_spherical(xyz, degrees=True):
    """
    Convert cartesian coordinates (x, y, z) to spherical (r, theta, phi).

    Parameters
    ----------
    xyz: (N, 3) ndarray
        Corresponding cartesian coordinates.
    degrees: bool, optional
        If True, azimuthal and polar will be returned in degrees.

    Returns
    -------
    radius: (N,) ndarray
        Radial distance.
    inclination: (N,) ndarray
        Polar angle.
    azimuth: (N,) ndarray
        Azimuthal angle.
    """
    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]

    radius = np.nan_to_num(np.sqrt((x * x) + (y * y) + (z * z)))

    inclination = np.nan_to_num(np.arccos(z / radius))

    azimuth = np.nan_to_num(np.arctan2(y, x))

    if degrees:
        inclination = np.rad2deg(inclination)
        azimuth = np.rad2deg(azimuth)

    return radius, inclination, azimuth


def cylindrical_to_cartesian(ro, phi, z, degrees=True):
    """
    Convert cylindrical coordinates (ro, phi, zeta) to cartesian (x, y, z).

    Parameters
    ----------
    ro: (N,) ndarray
        Radial distance.
    phi: (N,) ndarray
        Angular position.
    z: (N,) ndarray
        Altitude.
    degrees: bool, optional
        If True, angular will be assumed to be in degrees.

    Returns
    -------
    xyz: (N, 3) ndarray
        Corresponding cartesian coordinates.

    Notes
    -----
    The polar axis in the cylindrical system corresponds to the 'x' axis in the
    cartesian system.

    The longitudinal axis corresponds to the 'z' axis.
    """
    if degrees:
        phi = np.deg2rad(phi)

    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)

    xyz = np.empty((ro.shape[0], 3), dtype=np.float32)

    xyz[:, 0] = ro * cos_phi
    xyz[:, 1] = ro * sin_phi
    xyz[:, 2] = z

    return xyz


def cartesian_to_cylindrical(xyz, degrees=True):
    """
    Convert cartesian coordinates (x, y, z) to cylindrical (ro, phi, zeta).

    Parameters
    ----------
    xyz: (N, 3) ndarray
        Corresponding cartesian coordinates.
    degrees: bool, optional
        If True, azimuthal and polar will be returned in degrees.

    Returns
    -------
    radial_cylindrical: (N,) ndarray
        Radial distance.
    angular_cylindrical: (N,) ndarray
        Angular position.
    z: (N,) ndarray
        Altitude.

    Notes
    -----
    The polar axis in the cylindrical system corresponds to the 'x' axis in the
    cartesian system.

    The longitudinal axis corresponds to the 'z' axis.
    """
    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]

    radial_cylindrical = np.nan_to_num(np.sqrt((x * x) + (y * y)))

    angular_cylindrical = np.nan_to_num(np.arctan2(y, x))

    if degrees:
        angular_cylindrical = np.rad2deg(angular_cylindrical)

    return radial_cylindrical, angular_cylindrical, z


def cylindrical_to_spherical(ro, phi, zeta, degrees=True, phi_is_inclination=True):
    """
    Convert cylindrical coordinates (ro, phi, zeta) to spherical (r, theta, phi).

    Parameters
    ----------
    ro: (N,) ndarray
        Radial distance.
    phi: (N,) ndarray
        Angular position.
    zeta: (N,) ndarray
        Altitude.
    degrees: bool, optional
        If True, azimuthal and polar will be returned in degrees.
    phi_is_inclination: bool, optional
        See https://en.wikipedia.org/wiki/Cylindrical_coordinate_system#Spherical_coordinates.

    Returns
    -------
    r: (N,) ndarray
        Radial distance.
    theta: (N,) ndarray
        Azimuthal angle.
    phi: (N,) ndarray
        Polar angle.
    """

    r = np.sqrt((ro * ro) + (zeta * zeta))

    theta = phi

    if phi_is_inclination:
        phi = np.arctan2(ro, zeta)
    else:
        phi = np.arctan2(zeta, ro)

    if degrees:
        phi = np.rad2deg(phi)

    return r, theta, phi


def spherical_to_cylindrical(r, theta, phi, degrees=True, phi_is_inclination=False):
    """
    Convert spherical coordinates (r, theta, phi) to cylindrical (ro, phi, zeta).

    Parameters
    ----------
    r: (N,) ndarray
        Radial distance.
    theta: (N,) ndarray
        Azimuthal angle.
    phi: (N,) ndarray
        Polar angle.
    degrees: bool, optional
        If True, azimuthal and polar will be returned in degrees.
    phi_is_inclination: bool, optional
        See https://en.wikipedia.org/wiki/Cylindrical_coordinate_system#Spherical_coordinates.

    Returns
    -------
    ro: (N,) ndarray
        Radial distance.
    phi: (N,) ndarray
        Angular position.
    z: (N,) ndarray
        Altitude.
    """
    if degrees:
        phi = np.deg2rad(phi)

    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)

    ro = r * sin_phi
    z = r * cos_phi

    phi = theta

    return ro, phi, z
