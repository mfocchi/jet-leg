import numpy as np 
from scipy.spatial.transform import Rotation as R
from jet_leg.computational_geometry.math_tools import Math
import unittest

math = Math()

class TestMaths(unittest.TestCase):

    def rotation_matrix_from_vectors(self, vec1, vec2):
        """ Find the rotation matrix that aligns vec1 to vec2
        :param vec1: A 3d "source" vector
        :param vec2: A 3d "destination" vector
        :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
        """
        a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return rotation_matrix

    def test_rotationMatrixFromVector(self):
        n = np.array([0, 0, 1])
        Rotz = math.rotation_matrix_from_normal(n)
        np.testing.assert_array_equal(np.eye(3), Rotz)

        res = [[ 0.,  0.,  1.],
        [ 0., -1.,  0.],
        [ 1.,  0.,  0.]]
        Rotx = math.rotation_matrix_from_normal(np.array([1, 0, 0]))
        np.testing.assert_array_equal(res, Rotx)

        res = [[ 0.,  1.,  0.],
        [ 0., 0.,  1.],
        [ 1.,  0.,  0.]]
        Roty = math.rotation_matrix_from_normal(np.array([0, 1, 0]))
        np.testing.assert_array_equal(res, Roty)
