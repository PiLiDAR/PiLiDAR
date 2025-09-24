import numpy as np


def triangle_area(triangle):
    """ triangle is a (3,3) array such as triangle[i] is the ith vertex of the triangle
    """
    return 0.5 * np.linalg.norm(np.cross(triangle[1] - triangle[0],
                                         triangle[2] - triangle[0]))


def triangle_area_multi(v1, v2, v3):
    """ v1, v2, v3 are (N,3) arrays. Each one represents the vertices
    such as v1[i], v2[i], v3[i] represent the ith triangle
    """
    return 0.5 * np.linalg.norm(np.cross(v2 - v1,
                                         v3 - v1), axis=1)


def coplanar_area(points, plane_normal=None):
    """ Area of the coplanar polygon formed by the given points.

    Parameters
    ----------
    points: array
        The vertices of the selected area, the points are expected to be coplanar.
        Expected format:
            array([
            [x1,y1,z1],
            ...,
            [xn,yn,zn]])

    Returns
    -------
    area : float
        The area of the polygon formed by the given coplanar points.

    """

    if not plane_normal:

        #  p, normal = plane_def_by(points[:3])
        pass

    else:
        #  normal = normalize(plane_normal)
        pass

    #: get an array with the first point positioned as last
    points_rolled = np.roll(points, len(points) - 1, axis=0)

    cross_product = np.cross(points, points_rolled)

    summed = np.sum(cross_product, axis=0)

    total = np.dot(summed, plane_normal)

    area = 0.5 * abs(total)

    return area


def projected_area(points, plane_point, plane_normal):
    """ Area of the polygon formed by the points projected on the given plane.

    Parameters
    ----------
    points: array
        The vertices of the selected area.Expected format:
            array([
            [x1,y1,z1],
            ...,
            [xn,yn,zn]])


    Returns
    -------
    area : float
        The area of the polygon formed by the given coplanar points.

    """

    # points = project_on_plane(points, plane_point, plane_normal)

    area = coplanar_area(points, plane_normal=plane_normal)

    return area
