import unittest
import numpy as np
import quaternion
import rospy
from msckf import State, Quaternion


class TestNumpyQuaternion(unittest.TestCase):
    def test_multiply_by_unit_quaternion(self):
        id = np.quaternion(1, 0, 0, 0)

        q1 = np.quaternion(1, 2, 3, 4) # random
        self.assertEqual(id*q1, q1)
        self.assertEqual(q1*id, q1)

        q2 = np.quaternion(5, 6, 1000, 2000) # random
        self.assertEqual(id*q2, q2)
        self.assertEqual(q2*id, q2)

    def test_element_order(self):
        q = np.quaternion(1, 2, 3, 4) # random
        self.assertEqual(q.w, 1)
        self.assertEqual(q.x, 2)
        self.assertEqual(q.y, 3)
        self.assertEqual(q.z, 4)


class TestQuaternionToRotationMatrix(unittest.TestCase):
    def test_unit_quaternion_is_unit_matrix(self):
        q = np.quaternion(1, 0, 0, 0)
        m = Quaternion.to_rotation_matrix(q)
        np.testing.assert_allclose(m, np.eye(3))

    def test_non_unit_quaternion_is_non_unit_matrix(self):
        q = np.quaternion(3, 7, -1, 0.001).normalized() # random
        m = Quaternion.to_rotation_matrix(q)
        self.assertNotAlmostEqual(np.power(m - np.eye(3), 2).sum(), 0)

    def test_rotation_matrix_of_oposite_quaternion_is_the_same(self):
        q = np.quaternion(6.5, 3.2, 0, 0).normalized() # random
        q_mat = Quaternion.to_rotation_matrix(q)
        q_minus_mat = Quaternion.to_rotation_matrix(-1 * q)
        np.testing.assert_allclose(q_mat, q_minus_mat)

    def test_rotation_matrix_of_conjugate_quaternion_is_transposed_rotation_matrix(self):
        q = np.quaternion(9.4, -1.2, -4, 7.777).normalized() # random
        m = Quaternion.to_rotation_matrix(q).T
        m_from_conj = Quaternion.to_rotation_matrix(q.conjugate())
        np.testing.assert_allclose(m, m_from_conj)

    def test_rotation_matrix_of_composition_is_dot_product_of_rotation_matrices(self):
        q1 = np.quaternion(np.pi, 5.12, -np.pi, 2.56).normalized() # random
        q2 = np.quaternion(-np.pi, 0, 1.1, -2.2).normalized() # random

        m_of_composition = Quaternion.to_rotation_matrix(q1 * q2)
        m_from_q1 = Quaternion.to_rotation_matrix(q1)
        m_from_q2 = Quaternion.to_rotation_matrix(q2)
        m_from_dot = m_from_q1.dot(m_from_q2)
        np.testing.assert_allclose(m_of_composition, m_from_dot)

    def vector_to_quaternion_form_and_back_is_the_same_vector(self):
        vec = np.array([1, 2, 3]) # random
        vec_quat = Quaternion.vector_to_quaternion_form(vec)
        vec_back = Quaternion.vector_from_quaternion_form(vec_quat)
        np.testing.assert_allclose(vec, vec_back)

    def test_transformation_using_sandwich_and_rotation_matrix_are_same(self):
        q = np.quaternion(-1, -8, 4, 6).normalized() # random
        x = np.asarray([1.0, 2.0, 3.0]).reshape((3, 1)) # random
        x = x / x.sum()
        x_quat = Quaternion.vector_to_quaternion_form(x)
        x_next_quat_form = q * x_quat * q.conjugate()
        x_next_using_quaternions = Quaternion.vector_from_quaternion_form(x_next_quat_form)

        r = Quaternion.to_rotation_matrix(q)
        x_next_using_matrices = r.dot(x)

        np.testing.assert_allclose(x_next_using_quaternions, x_next_using_matrices)


class TestBasicMotion(unittest.TestCase):
    def test_no_movement(self):
        accel_item_first = (0.0, np.asarray([0.0, 0.0, -9.81]).reshape((3, 1)))
        gyro_item_second = (0.0, np.asarray([0.0, 0.0, 0.0]).reshape((3, 1)))
        initial_accel_mean = np.zeros((3, 1))
        state = State(accel_item_first, gyro_item_second, initial_accel_mean)
        accel_item_second = (1.0, np.asarray([0.0, 0.0, -9.81]).reshape((3, 1)))
        gyro_item_second = (1.0, np.asarray([0.0, 0.0, 0.0]).reshape((3, 1)))
        state = State.propagate(state, accel_item_second, gyro_item_second)
        self.assertEqual(state.q_B_G, np.quaternion(1.0, 0.0, 0.0, 0.0))
        np.testing.assert_allclose(state.p_B_G, np.zeros((3, 1)))
        np.testing.assert_allclose(state.v_B_G, np.zeros((3, 1)))

    def test_simple_move_in_x_direction(self):
        accel_item_first = (0.0, np.asarray([1.0, 0.0, -9.81]).reshape((3, 1)))
        gyro_item_second = (0.0, np.asarray([0.0, 0.0, 0.0]).reshape((3, 1)))
        initial_accel_mean = np.zeros((3, 1))
        state = State(accel_item_first, gyro_item_second, initial_accel_mean)
        accel_item_second = (1.0, np.asarray([1.0, 0.0, -9.81]).reshape((3, 1)))
        gyro_item_second = (1.0, np.asarray([0.0, 0.0, 0.0]).reshape((3, 1)))
        state = State.propagate(state, accel_item_second, gyro_item_second)
        self.assertEqual(state.q_B_G, np.quaternion(1.0, 0.0, 0.0, 0.0))
        np.testing.assert_allclose(state.p_B_G, np.asarray([0.5, 0.0, 0.0]).reshape((3, 1)))
        np.testing.assert_allclose(state.v_B_G, np.asarray([1.0, 0.0, 0.0]).reshape((3, 1)))

    def test_simple_rotation_about_x_axis(self):
        accel_item_first = (0.0, np.asarray([0.0, 0.0, -9.81]).reshape((3, 1)))
        gyro_item_second = (0.0, np.asarray([0.0, 0.0, 2.0*np.pi]).reshape((3, 1)))
        initial_accel_mean = np.zeros((3, 1))
        state = State(accel_item_first, gyro_item_second, initial_accel_mean)
        for i in range(1, 101):
            accel_item_second = (i*0.01, np.asarray([0.0, 0.0, -9.81]).reshape((3, 1)))
            gyro_item_second = (i*0.01, np.asarray([0.0, 0.0, 2.0*np.pi]).reshape((3, 1)))
            state = State.propagate(state, accel_item_second, gyro_item_second)
        np.testing.assert_allclose(state.q_B_G.components, np.array([-1.0, 0.0, 0.0, 0.0]), rtol=0, atol=1e-7)
        np.testing.assert_allclose(state.p_B_G, np.asarray([0.0, 0.0, 0.0]).reshape((3, 1)), rtol=0, atol=1e-7)
        np.testing.assert_allclose(state.v_B_G, np.asarray([0.0, 0.0, 0.0]).reshape((3, 1)), rtol=0, atol=1e-7)


class TestFrameTransformations(unittest.TestCase):
    def test_initial_setup(self):
        accel_item_first = (0.0, np.asarray([0.0, 0.0, -9.81]).reshape((3, 1)))
        gyro_item_second = (0.0, np.asarray([0.0, 0.0, 0.0]).reshape((3, 1)))
        initial_accel_mean = np.zeros((3, 1))
        state = State(accel_item_first, gyro_item_second, initial_accel_mean)
        state.augment()
        q_C_G = state.get_rotation_from_global_to_camera_pose(0)
        self.assertEqual(q_C_G, np.quaternion(1.0, 0.0, 0.0, 0.0))

    def test_rotation_of_camera_pose_after_rotation(self):
        accel_item_first = (0.0, np.asarray([0.0, 0.0, -9.81]).reshape((3, 1)))
        gyro_item_second = (0.0, np.asarray([0.0, 0.0, np.pi/2.0]).reshape((3, 1)))
        initial_accel_mean = np.zeros((3, 1))
        state = State(accel_item_first, gyro_item_second, initial_accel_mean)
        state.p_B_C = np.asarray([-1.0, 0.0, 0.0]).reshape((3, 1))
        for i in range(1, 101):
            acceleration_estimate = np.asarray([0.0, 0.0, -9.81]).reshape((3, 1))
            rotation_estimate = np.asarray([0.0, 0.0, np.pi/2.0]).reshape((3, 1))
            state = State.propagate(state, i*0.01, rotation_estimate, acceleration_estimate)
        state.augment()
        q_C_G = state.get_rotation_from_global_to_camera_pose(0)
        angles = State.quaternion_to_euler_angles(q_C_G)
        np.testing.assert_allclose(angles, np.array([0.0, 0.0, -np.pi/2.0]).reshape((3, 1)))
        p_C0_G = state.get_position_of_pose_in_global_frame(0)
        np.testing.assert_allclose(p_C0_G, np.array([0.0, -1.0, 0.0]).reshape((3, 1)), atol=1e-7)

if __name__ == '__main__':
    unittest.main()
