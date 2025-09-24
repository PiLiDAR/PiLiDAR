"""
This file is part of the pyquaternion python module

Author:         Kieran Wynn
Website:        https://github.com/KieranWynn/pyquaternion
Documentation:  http://kieranwynn.github.io/pyquaternion/

Version:         1.0.0
License:         The MIT License (MIT)

Copyright (c) 2015 Kieran Wynn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

quaternion.py - This file defines the core Quaternion class

"""

from __future__ import absolute_import, division, print_function # Add compatibility for Python 2.7+

from math import sqrt, pi, sin, cos, asin, acos, atan2, exp, log
from copy import deepcopy
import numpy as np # Numpy is required for many vector operations


class Quaternion:
    """Class to represent a 4-dimensional complex number or quaternion.

    Quaternion objects can be used generically as 4D numbers,
    or as unit quaternions to represent rotations in 3D space.

    Attributes:
        q: Quaternion 4-vector represented as a Numpy array

    """

    def __init__(self, *args, **kwargs):
        """Initialise a new Quaternion object.

        See Object Initialisation docs for complete behaviour:

        https://kieranwynn.github.io/pyquaternion/#object-initialisation

        """
        s = len(args)
        if s == 0:
            # No positional arguments supplied
            if kwargs:
                # Keyword arguments provided
                if ("scalar" in kwargs) or ("vector" in kwargs):
                    scalar = kwargs.get("scalar", 0.0)
                    if scalar is None:
                        scalar = 0.0
                    else:
                        scalar = float(scalar)

                    vector = kwargs.get("vector", [])
                    vector = self._validate_number_sequence(vector, 3)

                    self.q = np.hstack((scalar, vector))
                elif ("real" in kwargs) or ("imaginary" in kwargs):
                    real = kwargs.get("real", 0.0)
                    if real is None:
                        real = 0.0
                    else:
                        real = float(real)

                    imaginary = kwargs.get("imaginary", [])
                    imaginary = self._validate_number_sequence(imaginary, 3)

                    self.q = np.hstack((real, imaginary))
                elif ("axis" in kwargs) or ("radians" in kwargs) or ("degrees" in kwargs) or ("angle" in kwargs):
                    try:
                        axis = self._validate_number_sequence(kwargs["axis"], 3)
                    except KeyError:
                        raise ValueError(
                            "A valid rotation 'axis' parameter must be provided to describe a meaningful rotation."
                        )
                    angle = kwargs.get('radians') or self.to_radians(kwargs.get('degrees')) or kwargs.get('angle') or 0.0
                    self.q = Quaternion._from_axis_angle(axis, angle).q
                elif "array" in kwargs:
                    self.q = self._validate_number_sequence(kwargs["array"], 4)
                elif "matrix" in kwargs:
                    optional_args = {key: kwargs[key] for key in kwargs if key in ['rtol', 'atol']}
                    self.q = Quaternion._from_matrix(kwargs["matrix"], **optional_args).q
                else:
                    keys = sorted(kwargs.keys())
                    elements = [kwargs[kw] for kw in keys]
                    if len(elements) == 1:
                        r = float(elements[0])
                        self.q = np.array([r, 0.0, 0.0, 0.0])
                    else:
                        self.q = self._validate_number_sequence(elements, 4)

            else:
                # Default initialisation
                self.q = np.array([1.0, 0.0, 0.0, 0.0])
        elif s == 1:
            # Single positional argument supplied
            if isinstance(args[0], Quaternion):
                self.q = args[0].q
                return
            if args[0] is None:
                raise TypeError("Object cannot be initialised from {}".format(type(args[0])))
            try:
                r = float(args[0])
                self.q = np.array([r, 0.0, 0.0, 0.0])
                return
            except TypeError:
                pass  # If the single argument is not scalar, it should be a sequence

            self.q = self._validate_number_sequence(args[0], 4)
            return

        else:
            # More than one positional argument supplied
            self.q = self._validate_number_sequence(args, 4)

    def __hash__(self):
        return hash(tuple(self.q))

    def _validate_number_sequence(self, seq, n):
        """Validate a sequence to be of a certain length and ensure it's a numpy array of floats.

        Raises:
            ValueError: Invalid length or non-numeric value
        """
        if seq is None:
            return np.zeros(n)
        if len(seq) == n:
            try:
                l = [float(e) for e in seq]
            except ValueError:
                raise ValueError("One or more elements in sequence <{!r}> cannot be interpreted as a real number".format(seq))
            else:
                return np.asarray(l)
        elif len(seq) == 0:
            return np.zeros(n)
        else:
            raise ValueError("Unexpected number of elements in sequence. Got: {}, Expected: {}.".format(len(seq), n))

    # Initialise from matrix
    @classmethod
    def _from_matrix(cls, matrix, rtol=1e-05, atol=1e-08):
        """Initialise from matrix representation

        Create a Quaternion by specifying the 3x3 rotation or 4x4 transformation matrix
        (as a numpy array) from which the quaternion's rotation should be created.

        """
        try:
            shape = matrix.shape
        except AttributeError:
            raise TypeError("Invalid matrix type: Input must be a 3x3 or 4x4 numpy array or matrix")

        if shape == (3, 3):
            R = matrix
        elif shape == (4, 4):
            R = matrix[:-1][:,:-1] # Upper left 3x3 sub-matrix
        else:
            raise ValueError("Invalid matrix shape: Input must be a 3x3 or 4x4 numpy array or matrix")

        # Check matrix properties
        if not np.allclose(np.dot(R, R.conj().transpose()), np.eye(3), rtol=rtol, atol=atol):
            raise ValueError("Matrix must be orthogonal, i.e. its transpose should be its inverse")
        if not np.isclose(np.linalg.det(R), 1.0, rtol=rtol, atol=atol):
            raise ValueError("Matrix must be special orthogonal i.e. its determinant must be +1.0")

        def decomposition_method(matrix):
            """ Method supposedly able to deal with non-orthogonal matrices - NON-FUNCTIONAL!
            Based on this method: http://arc.aiaa.org/doi/abs/10.2514/2.4654
            """
            x, y, z = 0, 1, 2 # indices
            K = np.array([
                [R[x, x]-R[y, y]-R[z, z],  R[y, x]+R[x, y],           R[z, x]+R[x, z],           R[y, z]-R[z, y]],
                [R[y, x]+R[x, y],          R[y, y]-R[x, x]-R[z, z],   R[z, y]+R[y, z],           R[z, x]-R[x, z]],
                [R[z, x]+R[x, z],          R[z, y]+R[y, z],           R[z, z]-R[x, x]-R[y, y],   R[x, y]-R[y, x]],
                [R[y, z]-R[z, y],          R[z, x]-R[x, z],           R[x, y]-R[y, x],           R[x, x]+R[y, y]+R[z, z]]
            ])
            K = K / 3.0

            e_vals, e_vecs = np.linalg.eig(K)
            print('Eigenvalues:', e_vals)
            print('Eigenvectors:', e_vecs)
            max_index = np.argmax(e_vals)
            principal_component = e_vecs[max_index]
            return principal_component

        def trace_method(matrix):
            """
            This code uses a modification of the algorithm described in:
            https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
            which is itself based on the method described here:
            http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

            Altered to work with the column vector convention instead of row vectors
            """
            m = matrix.conj().transpose() # This method assumes row-vector and postmultiplication of that vector
            if m[2, 2] < 0:
                if m[0, 0] > m[1, 1]:
                    t = 1 + m[0, 0] - m[1, 1] - m[2, 2]
                    q = [m[1, 2]-m[2, 1],  t,  m[0, 1]+m[1, 0],  m[2, 0]+m[0, 2]]
                else:
                    t = 1 - m[0, 0] + m[1, 1] - m[2, 2]
                    q = [m[2, 0]-m[0, 2],  m[0, 1]+m[1, 0],  t,  m[1, 2]+m[2, 1]]
            else:
                if m[0, 0] < -m[1, 1]:
                    t = 1 - m[0, 0] - m[1, 1] + m[2, 2]
                    q = [m[0, 1]-m[1, 0],  m[2, 0]+m[0, 2],  m[1, 2]+m[2, 1],  t]
                else:
                    t = 1 + m[0, 0] + m[1, 1] + m[2, 2]
                    q = [t,  m[1, 2]-m[2, 1],  m[2, 0]-m[0, 2],  m[0, 1]-m[1, 0]]

            q = np.array(q).astype('float64')
            q *= 0.5 / sqrt(t)
            return q

        return cls(array=trace_method(R))

    # Initialise from axis-angle
    @classmethod
    def _from_axis_angle(cls, axis, angle):
        """Initialise from axis and angle representation

        Create a Quaternion by specifying the 3-vector rotation axis and rotation
        angle (in radians) from which the quaternion's rotation should be created.

        Params:
            axis: a valid numpy 3-vector
            angle: a real valued angle in radians
        """
        mag_sq = np.dot(axis, axis)
        if mag_sq == 0.0:
            raise ZeroDivisionError("Provided rotation axis has no length")
        # Ensure axis is in unit vector form
        if (abs(1.0 - mag_sq) > 1e-12):
            axis = axis / sqrt(mag_sq)
        theta = angle / 2.0
        r = cos(theta)
        i = axis * sin(theta)

        return cls(r, i[0], i[1], i[2])

    @classmethod
    def random(cls):
        """Generate a random unit quaternion.

        Uniformly distributed across the rotation space
        As per: http://planning.cs.uiuc.edu/node198.html
        """
        r1, r2, r3 = np.random.random(3)

        q1 = sqrt(1.0 - r1) * (sin(2 * pi * r2))
        q2 = sqrt(1.0 - r1) * (cos(2 * pi * r2))
        q3 = sqrt(r1)       * (sin(2 * pi * r3))
        q4 = sqrt(r1)       * (cos(2 * pi * r3))

        return cls(q1, q2, q3, q4)

    # Representation
    def __str__(self):
        """An informal, nicely printable string representation of the Quaternion object.
        """
        return "{:.3f} {:+.3f}i {:+.3f}j {:+.3f}k".format(self.q[0], self.q[1], self.q[2], self.q[3])

    def __repr__(self):
        """The 'official' string representation of the Quaternion object.

        This is a string representation of a valid Python expression that could be used
        to recreate an object with the same value (given an appropriate environment)
        """
        return "Quaternion({!r}, {!r}, {!r}, {!r})".format(self.q[0], self.q[1], self.q[2], self.q[3])

    def __format__(self, formatstr):
        """Inserts a customisable, nicely printable string representation of the Quaternion object

        The syntax for `format_spec` mirrors that of the built in format specifiers for floating point types.
        Check out the official Python [format specification mini-language](https://docs.python.org/3.4/library/string.html#formatspec) for details.
        """
        if formatstr.strip() == '': # Defualt behaviour mirrors self.__str__()
            formatstr = '+.3f'

        string = \
            "{:" + formatstr +"} "  + \
            "{:" + formatstr +"}i " + \
            "{:" + formatstr +"}j " + \
            "{:" + formatstr +"}k"
        return string.format(self.q[0], self.q[1], self.q[2], self.q[3])

    # Type Conversion
    def __int__(self):
        """Implements type conversion to int.

        Truncates the Quaternion object by only considering the real
        component and rounding to the next integer value towards zero.
        Note: to round to the closest integer, use int(round(float(q)))
        """
        return int(self.q[0])

    def __float__(self):
        """Implements type conversion to float.

        Truncates the Quaternion object by only considering the real
        component.
        """
        return float(self.q[0])

    def __complex__(self):
        """Implements type conversion to complex.

        Truncates the Quaternion object by only considering the real
        component and the first imaginary component.
        This is equivalent to a projection from the 4-dimensional hypersphere
        to the 2-dimensional complex plane.
        """
        return complex(self.q[0], self.q[1])

    def __bool__(self):
        return not (self == Quaternion(0.0))

    def __nonzero__(self):
        return not (self == Quaternion(0.0))

    def __invert__(self):
        return (self == Quaternion(0.0))

    # Comparison
    def __eq__(self, other):
        """Returns true if the following is true for each element:
        `absolute(a - b) <= (atol + rtol * absolute(b))`
        """
        if isinstance(other, Quaternion):
            r_tol = 1.0e-13
            a_tol = 1.0e-14
            try:
                isEqual = np.allclose(self.q, other.q, rtol=r_tol, atol=a_tol)
            except AttributeError:
                raise AttributeError("Error in internal quaternion representation means it cannot be compared like a numpy array.")
            return isEqual
        return self.__eq__(self.__class__(other))

    # Negation
    def __neg__(self):
        return self.__class__(array= -self.q)

    # Absolute value
    def __abs__(self):
        return self.norm

    # Addition
    def __add__(self, other):
        if isinstance(other, Quaternion):
            return self.__class__(array=self.q + other.q)
        return self + self.__class__(other)

    def __iadd__(self, other):
        return self + other

    def __radd__(self, other):
        return self + other

    # Subtraction
    def __sub__(self, other):
        return self + (-other)

    def __isub__(self, other):
        return self + (-other)

    def __rsub__(self, other):
        return -(self - other)

    # Multiplication
    def __mul__(self, other):
        if isinstance(other, Quaternion):
            return self.__class__(array=np.dot(self._q_matrix(), other.q))
        return self * self.__class__(other)

    def __imul__(self, other):
        return self * other

    def __rmul__(self, other):
        return self.__class__(other) * self

    def __matmul__(self, other):
        if isinstance(other, Quaternion):
            return self.q.__matmul__(other.q)
        return self.__matmul__(self.__class__(other))

    def __imatmul__(self, other):
        return self.__matmul__(other)

    def __rmatmul__(self, other):
        return self.__class__(other).__matmul__(self)

    # Division
    def __div__(self, other):
        if isinstance(other, Quaternion):
            if other == self.__class__(0.0):
                raise ZeroDivisionError("Quaternion divisor must be non-zero")
            return self * other.inverse
        return self.__div__(self.__class__(other))

    def __idiv__(self, other):
        return self.__div__(other)

    def __rdiv__(self, other):
        return self.__class__(other) * self.inverse

    def __truediv__(self, other):
        return self.__div__(other)

    def __itruediv__(self, other):
        return self.__idiv__(other)

    def __rtruediv__(self, other):
        return self.__rdiv__(other)

    # Exponentiation
    def __pow__(self, exponent):
        # source: https://en.wikipedia.org/wiki/Quaternion#Exponential.2C_logarithm.2C_and_power
        exponent = float(exponent) # Explicitly reject non-real exponents
        norm = self.norm
        if norm > 0.0:
            try:
                n, theta = self.polar_decomposition
            except ZeroDivisionError:
                # quaternion is a real number (no vector or imaginary part)
                return Quaternion(scalar=self.scalar ** exponent)
            return (self.norm ** exponent) * Quaternion(scalar=cos(exponent * theta), vector=(n * sin(exponent * theta)))
        return Quaternion(self)

    def __ipow__(self, other):
        return self ** other

    def __rpow__(self, other):
        return other ** float(self)

    # Quaternion Features
    def _vector_conjugate(self):
        return np.hstack((self.q[0], -self.q[1:4]))

    def _sum_of_squares(self):
        return np.dot(self.q, self.q)

    @property
    def conjugate(self):
        """Quaternion conjugate, encapsulated in a new instance.

        For a unit quaternion, this is the same as the inverse.

        Returns:
            A new Quaternion object clone with its vector part negated
        """
        return self.__class__(scalar=self.scalar, vector=-self.vector)

    @property
    def inverse(self):
        """Inverse of the quaternion object, encapsulated in a new instance.

        For a unit quaternion, this is the inverse rotation, i.e. when combined with the original rotation, will result in the null rotation.

        Returns:
            A new Quaternion object representing the inverse of this object
        """
        ss = self._sum_of_squares()
        if ss > 0:
            return self.__class__(array=(self._vector_conjugate() / ss))
        else:
            raise ZeroDivisionError("a zero quaternion (0 + 0i + 0j + 0k) cannot be inverted")

    @property
    def norm(self):
        """L2 norm of the quaternion 4-vector.

        This should be 1.0 for a unit quaternion (versor)
        Slow but accurate. If speed is a concern, consider using _fast_normalise() instead

        Returns:
            A scalar real number representing the square root of the sum of the squares of the elements of the quaternion.
        """
        mag_squared = self._sum_of_squares()
        return sqrt(mag_squared)

    @property
    def magnitude(self):
        return self.norm

    def _normalise(self):
        """Object is guaranteed to be a unit quaternion after calling this
        operation UNLESS the object is equivalent to Quaternion(0)
        """
        if not self.is_unit():
            n = self.norm
            if n > 0:
                self.q = self.q / n

    def _fast_normalise(self):
        """Normalise the object to a unit quaternion using a fast approximation method if appropriate.

        Object is guaranteed to be a quaternion of approximately unit length
        after calling this operation UNLESS the object is equivalent to Quaternion(0)
        """
        if not self.is_unit():
            mag_squared = np.dot(self.q, self.q)
            if (mag_squared == 0):
                return
            if (abs(1.0 - mag_squared) < 2.107342e-08):
                mag =  ((1.0 + mag_squared) / 2.0) # More efficient. Pade approximation valid if error is small
            else:
                mag =  sqrt(mag_squared) # Error is too big, take the performance hit to calculate the square root properly

            self.q = self.q / mag

    @property
    def normalised(self):
        """Get a unit quaternion (versor) copy of this Quaternion object.

        A unit quaternion has a `norm` of 1.0

        Returns:
            A new Quaternion object clone that is guaranteed to be a unit quaternion
        """
        q = Quaternion(self)
        q._normalise()
        return q

    @property
    def polar_unit_vector(self):
        vector_length = np.linalg.norm(self.vector)
        if vector_length <= 0.0:
            raise ZeroDivisionError('Quaternion is pure real and does not have a unique unit vector')
        return self.vector / vector_length

    @property
    def polar_angle(self):
         return acos(self.scalar / self.norm)

    @property
    def polar_decomposition(self):
        """
        Returns the unit vector and angle of a non-scalar quaternion according to the following decomposition

        q =  q.norm() * (e ** (q.polar_unit_vector * q.polar_angle))

        source: https://en.wikipedia.org/wiki/Polar_decomposition#Quaternion_polar_decomposition
        """
        return self.polar_unit_vector, self.polar_angle

    @property
    def unit(self):
        return self.normalised

    def is_unit(self, tolerance=1e-14):
        """Determine whether the quaternion is of unit length to within a specified tolerance value.

        Params:
            tolerance: [optional] maximum absolute value by which the norm can differ from 1.0 for the object to be considered a unit quaternion. Defaults to `1e-14`.

        Returns:
            `True` if the Quaternion object is of unit length to within the specified tolerance value. `False` otherwise.
        """
        return abs(1.0 - self._sum_of_squares()) < tolerance # if _sum_of_squares is 1, norm is 1. This saves a call to sqrt()

    def _q_matrix(self):
        """Matrix representation of quaternion for multiplication purposes.
        """
        return np.array([
            [self.q[0], -self.q[1], -self.q[2], -self.q[3]],
            [self.q[1],  self.q[0], -self.q[3],  self.q[2]],
            [self.q[2],  self.q[3],  self.q[0], -self.q[1]],
            [self.q[3], -self.q[2],  self.q[1],  self.q[0]]])

    def _q_bar_matrix(self):
        """Matrix representation of quaternion for multiplication purposes.
        """
        return np.array([
            [self.q[0], -self.q[1], -self.q[2], -self.q[3]],
            [self.q[1],  self.q[0],  self.q[3], -self.q[2]],
            [self.q[2], -self.q[3],  self.q[0],  self.q[1]],
            [self.q[3],  self.q[2], -self.q[1],  self.q[0]]])

    def _rotate_quaternion(self, q):
        """Rotate a quaternion vector using the stored rotation.

        Params:
            q: The vector to be rotated, in quaternion form (0 + xi + yj + kz)

        Returns:
            A Quaternion object representing the rotated vector in quaternion from (0 + xi + yj + kz)
        """
        self._normalise()
        return self * q * self.conjugate

    def rotate(self, vector):
        """Rotate a 3D vector by the rotation stored in the Quaternion object.

        Params:
            vector: A 3-vector specified as any ordered sequence of 3 real numbers corresponding to x, y, and z values.
                Some types that are recognised are: numpy arrays, lists and tuples.
                A 3-vector can also be represented by a Quaternion object who's scalar part is 0 and vector part is the required 3-vector.
                Thus it is possible to call `Quaternion.rotate(q)` with another quaternion object as an input.

        Returns:
            The rotated vector returned as the same type it was specified at input.

        Raises:
            TypeError: if any of the vector elements cannot be converted to a real number.
            ValueError: if `vector` cannot be interpreted as a 3-vector or a Quaternion object.

        """
        if isinstance(vector, Quaternion):
            return self._rotate_quaternion(vector)
        q = Quaternion(vector=vector)
        a = self._rotate_quaternion(q).vector
        if isinstance(vector, list):
            l = [x for x in a]
            return l
        elif isinstance(vector, tuple):
            l = [x for x in a]
            return tuple(l)
        else:
            return a

    @classmethod
    def exp(cls, q):
        """Quaternion Exponential.

        Find the exponential of a quaternion amount.

        Params:
             q: the input quaternion/argument as a Quaternion object.

        Returns:
             A quaternion amount representing the exp(q). See [Source](https://math.stackexchange.com/questions/1030737/exponential-function-of-quaternion-derivation for more information and mathematical background).

        Note:
             The method can compute the exponential of any quaternion.
        """
        tolerance = 1e-17
        v_norm = np.linalg.norm(q.vector)
        vec = q.vector
        if v_norm > tolerance:
            vec = vec / v_norm
        magnitude = exp(q.scalar)
        return Quaternion(scalar = magnitude * cos(v_norm), vector = magnitude * sin(v_norm) * vec)

    @classmethod
    def log(cls, q):
        """Quaternion Logarithm.

        Find the logarithm of a quaternion amount.

        Params:
             q: the input quaternion/argument as a Quaternion object.

        Returns:
             A quaternion amount representing log(q) := (log(|q|), v/|v|acos(w/|q|)).

        Note:
            The method computes the logarithm of general quaternions. See [Source](https://math.stackexchange.com/questions/2552/the-logarithm-of-quaternion/2554#2554) for more details.
        """
        v_norm = np.linalg.norm(q.vector)
        q_norm = q.norm
        tolerance = 1e-17
        if q_norm < tolerance:
            # 0 quaternion - undefined
            return Quaternion(scalar=-float('inf'), vector=float('nan')*q.vector)
        if v_norm < tolerance:
            # real quaternions - no imaginary part
            return Quaternion(scalar=log(q_norm), vector=[0, 0, 0])
        vec = q.vector / v_norm
        return Quaternion(scalar=log(q_norm), vector=acos(q.scalar/q_norm)*vec)

    @classmethod
    def exp_map(cls, q, eta):
        """Quaternion exponential map.

        Find the exponential map on the Riemannian manifold described
        by the quaternion space.

        Params:
             q: the base point of the exponential map, i.e. a Quaternion object
           eta: the argument of the exponential map, a tangent vector, i.e. a Quaternion object

        Returns:
            A quaternion p such that p is the endpoint of the geodesic starting at q
            in the direction of eta, having the length equal to the magnitude of eta.

        Note:
            The exponential map plays an important role in integrating orientation
            variations (e.g. angular velocities). This is done by projecting
            quaternion tangent vectors onto the quaternion manifold.
        """
        return q * Quaternion.exp(eta)

    @classmethod
    def sym_exp_map(cls, q, eta):
        """Quaternion symmetrized exponential map.

        Find the symmetrized exponential map on the quaternion Riemannian
        manifold.

        Params:
             q: the base point as a Quaternion object
           eta: the tangent vector argument of the exponential map
                as a Quaternion object

        Returns:
            A quaternion p.

        Note:
            The symmetrized exponential formulation is akin to the exponential
            formulation for symmetric positive definite tensors [Source](http://www.academia.edu/7656761/On_the_Averaging_of_Symmetric_Positive-Definite_Tensors)
        """
        sqrt_q = q ** 0.5
        return sqrt_q * Quaternion.exp(eta) * sqrt_q

    @classmethod
    def log_map(cls, q, p):
        """Quaternion logarithm map.

        Find the logarithm map on the quaternion Riemannian manifold.

        Params:
             q: the base point at which the logarithm is computed, i.e.
                a Quaternion object
             p: the argument of the quaternion map, a Quaternion object

        Returns:
            A tangent vector having the length and direction given by the
            geodesic joining q and p.
        """
        return Quaternion.log(q.inverse * p)

    @classmethod
    def sym_log_map(cls, q, p):
        """Quaternion symmetrized logarithm map.

        Find the symmetrized logarithm map on the quaternion Riemannian manifold.

        Params:
             q: the base point at which the logarithm is computed, i.e.
                a Quaternion object
             p: the argument of the quaternion map, a Quaternion object

        Returns:
            A tangent vector corresponding to the symmetrized geodesic curve formulation.

        Note:
            Information on the symmetrized formulations given in [Source](https://www.researchgate.net/publication/267191489_Riemannian_L_p_Averaging_on_Lie_Group_of_Nonzero_Quaternions).
        """
        inv_sqrt_q = (q ** (-0.5))
        return Quaternion.log(inv_sqrt_q * p * inv_sqrt_q)

    @classmethod
    def absolute_distance(cls, q0, q1):
        """Quaternion absolute distance.

        Find the distance between two quaternions accounting for the sign ambiguity.

        Params:
            q0: the first quaternion
            q1: the second quaternion

        Returns:
           A positive scalar corresponding to the chord of the shortest path/arc that
           connects q0 to q1.

        Note:
           This function does not measure the distance on the hypersphere, but
           it takes into account the fact that q and -q encode the same rotation.
           It is thus a good indicator for rotation similarities.
        """
        q0_minus_q1 = q0 - q1
        q0_plus_q1  = q0 + q1
        d_minus = q0_minus_q1.norm
        d_plus  = q0_plus_q1.norm
        if d_minus < d_plus:
            return d_minus
        else:
            return d_plus

    @classmethod
    def distance(cls, q0, q1):
        """Quaternion intrinsic distance.

        Find the intrinsic geodesic distance between q0 and q1.

        Params:
            q0: the first quaternion
            q1: the second quaternion

        Returns:
           A positive amount corresponding to the length of the geodesic arc
           connecting q0 to q1.

        Note:
           Although the q0^(-1)*q1 != q1^(-1)*q0, the length of the path joining
           them is given by the logarithm of those product quaternions, the norm
           of which is the same.
        """
        q = Quaternion.log_map(q0, q1)
        return q.norm

    @classmethod
    def sym_distance(cls, q0, q1):
        """Quaternion symmetrized distance.

        Find the intrinsic symmetrized geodesic distance between q0 and q1.

        Params:
            q0: the first quaternion
            q1: the second quaternion

        Returns:
           A positive amount corresponding to the length of the symmetrized
           geodesic curve connecting q0 to q1.

        Note:
           This formulation is more numerically stable when performing
           iterative gradient descent on the Riemannian quaternion manifold.
           However, the distance between q and -q is equal to pi, rendering this
           formulation not useful for measuring rotation similarities when the
           samples are spread over a "solid" angle of more than pi/2 radians
           (the spread refers to quaternions as point samples on the unit hypersphere).
        """
        q = Quaternion.sym_log_map(q0, q1)
        return q.norm

    @classmethod
    def slerp(cls, q0, q1, amount=0.5):
        """Spherical Linear Interpolation between quaternions.
        Implemented as described in https://en.wikipedia.org/wiki/Slerp

        Find a valid quaternion rotation at a specified distance along the
        minor arc of a great circle passing through any two existing quaternion
        endpoints lying on the unit radius hypersphere.

        This is a class method and is called as a method of the class itself rather than on a particular instance.

        Params:
            q0: first endpoint rotation as a Quaternion object
            q1: second endpoint rotation as a Quaternion object
            amount: interpolation parameter between 0 and 1. This describes the linear placement position of
                the result along the arc between endpoints; 0 being at `q0` and 1 being at `q1`.
                Defaults to the midpoint (0.5).

        Returns:
            A new Quaternion object representing the interpolated rotation. This is guaranteed to be a unit quaternion.

        Note:
            This feature only makes sense when interpolating between unit quaternions (those lying on the unit radius hypersphere).
                Calling this method will implicitly normalise the endpoints to unit quaternions if they are not already unit length.
        """
        # Ensure quaternion inputs are unit quaternions and 0 <= amount <=1
        q0._fast_normalise()
        q1._fast_normalise()
        amount = np.clip(amount, 0, 1)

        dot = np.dot(q0.q, q1.q)

        # If the dot product is negative, slerp won't take the shorter path.
        # Note that v1 and -v1 are equivalent when the negation is applied to all four components.
        # Fix by reversing one quaternion
        if dot < 0.0:
            q0.q = -q0.q
            dot = -dot

        # sin_theta_0 can not be zero
        if dot > 0.9995:
            qr = Quaternion(q0.q + amount * (q1.q - q0.q))
            qr._fast_normalise()
            return qr

        theta_0 = np.arccos(dot)  # Since dot is in range [0, 0.9995], np.arccos() is safe
        sin_theta_0 = np.sin(theta_0)

        theta = theta_0 * amount
        sin_theta = np.sin(theta)

        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        qr = Quaternion((s0 * q0.q) + (s1 * q1.q))
        qr._fast_normalise()
        return qr

    @classmethod
    def intermediates(cls, q0, q1, n, include_endpoints=False):
        """Generator method to get an iterable sequence of `n` evenly spaced quaternion
        rotations between any two existing quaternion endpoints lying on the unit
        radius hypersphere.

        This is a convenience function that is based on `Quaternion.slerp()` as defined above.

        This is a class method and is called as a method of the class itself rather than on a particular instance.

        Params:
            q_start: initial endpoint rotation as a Quaternion object
            q_end:   final endpoint rotation as a Quaternion object
            n:       number of intermediate quaternion objects to include within the interval
            include_endpoints: [optional] if set to `True`, the sequence of intermediates
                will be 'bookended' by `q_start` and `q_end`, resulting in a sequence length of `n + 2`.
                If set to `False`, endpoints are not included. Defaults to `False`.

        Yields:
            A generator object iterating over a sequence of intermediate quaternion objects.

        Note:
            This feature only makes sense when interpolating between unit quaternions (those lying on the unit radius hypersphere).
            Calling this method will implicitly normalise the endpoints to unit quaternions if they are not already unit length.
        """
        step_size = 1.0 / (n + 1)
        if include_endpoints:
            steps = [i * step_size for i in range(0, n + 2)]
        else:
            steps = [i * step_size for i in range(1, n + 1)]
        for step in steps:
            yield cls.slerp(q0, q1, step)

    def derivative(self, rate):
        """Get the instantaneous quaternion derivative representing a quaternion rotating at a 3D rate vector `rate`

        Params:
            rate: numpy 3-array (or array-like) describing rotation rates about the global x, y and z axes respectively.

        Returns:
            A unit quaternion describing the rotation rate
        """
        rate = self._validate_number_sequence(rate, 3)
        return 0.5 * self * Quaternion(vector=rate)

    def integrate(self, rate, timestep):
        """Advance a time varying quaternion to its value at a time `timestep` in the future.

        The Quaternion object will be modified to its future value.
        It is guaranteed to remain a unit quaternion.

        Params:

        rate: numpy 3-array (or array-like) describing rotation rates about the
            global x, y and z axes respectively.
        timestep: interval over which to integrate into the future.
            Assuming *now* is `T=0`, the integration occurs over the interval
            `T=0` to `T=timestep`. Smaller intervals are more accurate when
            `rate` changes over time.

        Note:
            The solution is closed form given the assumption that `rate` is constant
            over the interval of length `timestep`.
        """
        self._fast_normalise()
        rate = self._validate_number_sequence(rate, 3)

        rotation_vector = rate * timestep
        rotation_norm = np.linalg.norm(rotation_vector)
        if rotation_norm > 0:
            axis = rotation_vector / rotation_norm
            angle = rotation_norm
            q2 = Quaternion(axis=axis, angle=angle)
            self.q = (self * q2).q
            self._fast_normalise()


    @property
    def rotation_matrix(self):
        """Get the 3x3 rotation matrix equivalent of the quaternion rotation.

        Returns:
            A 3x3 orthogonal rotation matrix as a 3x3 Numpy array

        Note:
            This feature only makes sense when referring to a unit quaternion. Calling this method will implicitly normalise the Quaternion object to a unit quaternion if it is not already one.

        """
        self._normalise()
        product_matrix = np.dot(self._q_matrix(), self._q_bar_matrix().conj().transpose())
        return product_matrix[1:][:, 1:]

    @property
    def transformation_matrix(self):
        """Get the 4x4 homogeneous transformation matrix equivalent of the quaternion rotation.

        Returns:
            A 4x4 homogeneous transformation matrix as a 4x4 Numpy array

        Note:
            This feature only makes sense when referring to a unit quaternion. Calling this method will implicitly normalise the Quaternion object to a unit quaternion if it is not already one.
        """
        t = np.array([[0.0], [0.0], [0.0]])
        Rt = np.hstack([self.rotation_matrix, t])
        return np.vstack([Rt, np.array([0.0, 0.0, 0.0, 1.0])])

    @property
    def yaw_pitch_roll(self):
        """Get the equivalent yaw-pitch-roll angles aka. intrinsic Tait-Bryan angles following the z-y'-x'' convention

        Returns:
            yaw:    rotation angle around the z-axis in radians, in the range `[-pi, pi]`
            pitch:  rotation angle around the y'-axis in radians, in the range `[-pi/2, -pi/2]`
            roll:   rotation angle around the x''-axis in radians, in the range `[-pi, pi]`

        The resulting rotation_matrix would be R = R_x(roll) R_y(pitch) R_z(yaw)

        Note:
            This feature only makes sense when referring to a unit quaternion. Calling this method will implicitly normalise the Quaternion object to a unit quaternion if it is not already one.
        """

        self._normalise()
        yaw = np.arctan2(2 * (self.q[0] * self.q[3] - self.q[1] * self.q[2]),
            1 - 2 * (self.q[2] ** 2 + self.q[3] ** 2))
        pitch = np.arcsin(2 * (self.q[0] * self.q[2] + self.q[3] * self.q[1]))
        roll = np.arctan2(2 * (self.q[0] * self.q[1] - self.q[2] * self.q[3]),
            1 - 2 * (self.q[1] ** 2 + self.q[2] ** 2))

        return yaw, pitch, roll

    def _wrap_angle(self, theta):
        """Helper method: Wrap any angle to lie between -pi and pi

        Odd multiples of pi are wrapped to +pi (as opposed to -pi)
        """
        result = ((theta + pi) % (2 * pi)) - pi
        if result == -pi:
            result = pi
        return result

    def get_axis(self, undefined=np.zeros(3)):
        """Get the axis or vector about which the quaternion rotation occurs

        For a null rotation (a purely real quaternion), the rotation angle will
        always be `0`, but the rotation axis is undefined.
        It is by default assumed to be `[0, 0, 0]`.

        Params:
            undefined: [optional] specify the axis vector that should define a null rotation.
                This is geometrically meaningless, and could be any of an infinite set of vectors,
                but can be specified if the default (`[0, 0, 0]`) causes undesired behaviour.

        Returns:
            A Numpy unit 3-vector describing the Quaternion object's axis of rotation.

        Note:
            This feature only makes sense when referring to a unit quaternion.
            Calling this method will implicitly normalise the Quaternion object to a unit quaternion if it is not already one.
        """
        tolerance = 1e-17
        self._normalise()
        norm = np.linalg.norm(self.vector)
        if norm < tolerance:
            # Here there are an infinite set of possible axes, use what has been specified as an undefined axis.
            return undefined
        else:
            return self.vector / norm

    @property
    def axis(self):
        return self.get_axis()

    @property
    def angle(self):
        """Get the angle (in radians) describing the magnitude of the quaternion rotation about its rotation axis.

        This is guaranteed to be within the range (-pi:pi) with the direction of
        rotation indicated by the sign.

        When a particular rotation describes a 180 degree rotation about an arbitrary
        axis vector `v`, the conversion to axis / angle representation may jump
        discontinuously between all permutations of `(-pi, pi)` and `(-v, v)`,
        each being geometrically equivalent (see Note in documentation).

        Returns:
            A real number in the range (-pi:pi) describing the angle of rotation
                in radians about a Quaternion object's axis of rotation.

        Note:
            This feature only makes sense when referring to a unit quaternion.
            Calling this method will implicitly normalise the Quaternion object to a unit quaternion if it is not already one.
        """
        self._normalise()
        norm = np.linalg.norm(self.vector)
        return self._wrap_angle(2.0 * atan2(norm, self.scalar))

    @property
    def degrees(self):
        return self.to_degrees(self.angle)

    @property
    def radians(self):
        return self.angle

    @property
    def scalar(self):
        """ Return the real or scalar component of the quaternion object.

        Returns:
            A real number i.e. float
        """
        return self.q[0]

    @property
    def vector(self):
        """ Return the imaginary or vector component of the quaternion object.

        Returns:
            A numpy 3-array of floats. NOT guaranteed to be a unit vector
        """
        return self.q[1:4]

    @property
    def real(self):
        return self.scalar

    @property
    def imaginary(self):
        return self.vector

    @property
    def w(self):
        return self.q[0]

    @property
    def x(self):
        return self.q[1]

    @property
    def y(self):
        return self.q[2]

    @property
    def z(self):
        return self.q[3]

    @property
    def elements(self):
        """ Return all the elements of the quaternion object.

        Returns:
            A numpy 4-array of floats. NOT guaranteed to be a unit vector
        """
        return self.q

    def __getitem__(self, index):
        index = int(index)
        return self.q[index]

    def __setitem__(self, index, value):
        index = int(index)
        self.q[index] = float(value)

    def __copy__(self):
        result = self.__class__(self.q)
        return result

    def __deepcopy__(self, memo):
        result = self.__class__(deepcopy(self.q, memo))
        memo[id(self)] = result
        return result

    @staticmethod
    def to_degrees(angle_rad):
        if angle_rad is not None:
            return float(angle_rad) / pi * 180.0

    @staticmethod
    def to_radians(angle_deg):
        if angle_deg is not None:
            return float(angle_deg) / 180.0 * pi
