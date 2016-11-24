import rospy
import heapq
import tf2_ros
import cv2
import numpy as np
import sys
import bisect
import quaternion
import operator
import threading
import yaml
import scipy
import scipy.linalg
import scipy.stats
import traceback
import pickle as pkl
from enum import Enum
from numpy.linalg import inv
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, Image, PointCloud, ChannelFloat32
from geometry_msgs.msg import Vector3, Point32, TransformStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from cv_bridge import CvBridge, CvBridgeError
from scipy.optimize.slsqp import approx_jacobian


def null(A, eps=1e-12):
    u, s, vh = scipy.linalg.svd(A)
    padding = max(0,np.shape(A)[1]-np.shape(s)[0])
    null_mask = np.concatenate(((s <= eps), np.ones((padding,),dtype=bool)),axis=0)
    null_space = scipy.compress(null_mask, vh, axis=0)
    return scipy.transpose(null_space)


def print_yellow(msg, *args, **kwargs):
    print('\033[0;33m' + msg + '\033[0m')


def print_yellow_bold(msg, *args, **kwargs):
    print('\033[1;33m' + msg + '\033[0m')


class Quaternion:
    @staticmethod
    def to_rotation_matrix(q):
        assert abs(q.norm() - 1) < 1e-6, "Expected unit quaternion. Norm is {:.6f}".format(q.norm())
        return np.array([
            [q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z,             2*(q.x*q.y-q.w*q.z),             2*(q.x*q.z+q.w*q.y)],
            [            2*(q.x*q.y+q.w*q.z), q.w*q.w-q.x*q.x+q.y*q.y-q.z*q.z,             2*(q.y*q.z-q.w*q.x)],
            [            2*(q.x*q.z-q.w*q.y),             2*(q.y*q.z+q.w*q.x), q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z]
        ])

    @staticmethod
    def to_euler_angles(q):
        assert abs(q.norm() - 1) < 1e-6, "Expected unit quaternion. Norm is {:.6f}".format(q.norm())
        return np.array([
            np.arctan2(2*(-q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y)),
            np.arcsin(2*(-q.x*q.y - q.z*q.x)),
            np.arctan2(2*(-q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        ]).reshape((3, 1))

    @staticmethod
    def vector_to_quaternion_form(vec):
        "Convert vector to quaternion form. This is used for sandwich product."
        assert vec.shape == (3, 1), "Expected 3D vector"
        return np.quaternion(0.0, vec[0], vec[1], vec[2])

    @staticmethod
    def vector_from_quaternion_form(vec_quat):
        "Convert vectom from quaternion form. This is used for sandwich product."
        assert type(vec_quat) == np.quaternion, "Expected quaternion"
        assert vec_quat.w == 0, "vec_quat is not a quaternion form of 3D vector."
        return vec_quat.vec.reshape((3, 1))

    @staticmethod
    def cross_matrix(vec):
        "Get $a \to [a]_\times$"
        assert len(vec) == 3, "Expected vector of length 3"
        a_x, a_y, a_z = vec
        return np.asarray([
            [0, -1*a_z, a_y],
            [a_z, 0, -1*a_x],
            [-1*a_y, a_x, 0]
        ])
    
    @staticmethod
    def big_omega_matrix(vec):
        assert len(vec) == 3, "Expected vector of length 3"
        m = np.zeros((4, 4))
        m[1:, 1:] = -1 * Quaternion.cross_matrix(vec)
        m[0:1, 1:] = -1 * vec.T
        m[1:, 0:1] = vec
        return m


class ImuDevice(Enum):
    accelerometer = 1
    gyroscope = 2


class ImuItem:
    def __init__(self, device, time, value):
        assert type(device) == ImuDevice
        assert isinstance(time, float)
        assert time >= 0.0
        assert value.shape == (3, 1)

        self._device = device
        self._time = time
        self._value = value

    def __lt__(self, other):
        return self._compare(other, operator.lt)

    def __le__(self, other):
        return self._compare(other, operator.le)

    def __gt__(self, other):
        return self._compare(other, operator.gt)

    def __ge__(self, other):
        return self._compare(other, operator.ge)

    def __eq__(self, other):
        return self._compare(other, operator.eq)

    def __ne__(self, other):
        return self._compare(other, operator.ne)

    def _compare(self, other, op):
        if isinstance(other, ImuItem): return op(self.time, other.time)
        elif isinstance(other, float): return op(self.time, other)
        else: return NotImplemented

    @property
    def device(self):
        return self._device

    @property
    def time(self):
        return self._time

    @property
    def value(self):
        return self._value


class CameraItem:
    def __init__(self, time, image):
        self.time = time
        self.image = image


class ImuBuffer:
    """
    Contains a list of IMU measurements. Times are unique.
    """
    def __init__(self, device):
        assert type(device) == ImuDevice
        self.device = device
        self.items = []
    
    def push(self, imu_item):
        assert isinstance(imu_item, ImuItem), 'Expected type `ImuItem`. Got {}.'.format(type(imu_item))
        assert imu_item.device == self.device
        pos = bisect.bisect_left(self.items, imu_item)
        if (len(self.items) > 0) and (pos != len(self.items)):
            assert self.items[pos + 1] != imu_item, "IMU item already in buffer for time {}".format(imu_item.time)
        self.items.insert(pos, imu_item)
        self.check_integrity()
        
    def check_integrity(self):
        assert all(self.items[i] <= self.items[i+1] for i in range(len(self.items)-1)), 'Buffer is not sorted.'
        assert all(self.items[i] != self.items[i+1] for i in range(len(self.items)-1)), 'Buffer contains duplicates.'

    @property
    def min(self):
        if len(self.items) == 0:
            return None
        else:
            return self.items[0]

    @property
    def max(self):
        if len(self.items) == 0:
            return None
        else:
            return self.items[-1]

    @property
    def mean_value(self):
        if len(self.items) == 0:
            return None
        s = np.zeros((3, 1))
        for item in self.items:
            s += item.value
        return s / len(self.items)

    def __len__(self):
        return len(self.items)

    def __getitem__(self, ix):
        assert ix >= 0
        assert ix < len(self.items)
        return self.items[ix]

    def pop_min(self):
        assert len(self.items) > 0
        return self.items.pop(0)

    def interpolate(self, time):
        assert isinstance(time, float), 'Expected `float`. Got {}.'.format(type(time))
        assert len(self.items) > 0
        assert time >= self.min.time, 'Interpolation into past {}. Min time {}.'.format(time, self.min.time)
        assert time <= self.max.time, 'Interpolation into future {}. Max time {}.'.format(time, self.max.time)
        self.check_integrity()
        pos = bisect.bisect_left(self.items, time)
        if pos == 0: return self.min
        if pos == len(self.items): return self.max
        assert (self.items[pos-1].time < time) and (time <= self.items[pos].time), 'Expected {:.4f} < {:.4f} <= {:.4f} at {} out of {} ({} and {}).'.format(self.items[pos-1].time, time, self.items[pos].time, pos, len(self.items), self.items[pos-1].time < time, time <= self.items[pos].time)
        a = self.items[pos-1].time
        b = self.items[pos].time
        t = (time-a)/(b-a)
        assert (t >= 0.0) and (t <= 1.0)
        value = t*self.items[pos-1].value + (1-t)*self.items[pos].value
        return ImuItem(self.device, time, value)


class PoseGet():
    def __init__(self, state):
        self.state = state
                
    def __getitem__(self, key):
        return self.state.camera_poses[key - self.state.first_camera_pose_id]


class BodyState:
    global_gravity = np.asarray([0.0, 0.0, 0]).reshape((3, 1))

    def __init__(self):
        self.q_B_G = None
        self.p_B_G = None
        self.v_B_G = None
        self.time = None
        self.rotation_estimate = None
        self.acceleration_estimate = None

    def time_to(self, other):
        assert isinstance(other, BodyState)
        return other.time - self.time

    @staticmethod
    def initialize(time, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G):
        state = BodyState()
        state.q_B_G = q_B_G
        state.p_B_G = p_B_G
        state.v_B_G = v_B_G
        state.time = time
        state.rotation_estimate = rotation_estimate
        state.acceleration_estimate = acceleration_estimate
        return state

    @staticmethod
    def copy(other):
        state = BodyState()
        state.q_B_G = other.q_B_G
        state.p_B_G = other.p_B_G
        state.v_B_G = other.v_B_G
        state.time = other.time
        state.rotation_estimate = other.rotation_estimate
        state.acceleration_estimate = other.acceleration_estimate
        return state

    @staticmethod
    def propagate(from_state, time, rotation_estimate, acceleration_estimate):
        to_state = BodyState.copy(from_state)
        to_state.time = time
        to_state.rotation_estimate = rotation_estimate
        to_state.acceleration_estimate = acceleration_estimate
        q_Bnext_Bcurrent = BodyState._propagate_gyro(from_state, to_state)
        p_delta, v_delta = BodyState._propagate_accel(from_state, to_state, q_Bnext_Bcurrent)

        to_state.q_B_G = from_state.q_B_G * q_Bnext_Bcurrent
        to_state.p_B_G = from_state.p_B_G + p_delta
        to_state.v_B_G = from_state.v_B_G + v_delta
        return to_state

    @staticmethod
    def _propagate_gyro( from_state, to_state):
        gyro_est = to_state.rotation_estimate
        gyro_est_prev = from_state.rotation_estimate
        gyro_est_mid = 0.5*(gyro_est + gyro_est_prev)
        q_0 = np.asarray([1.0, 0.0, 0.0, 0.0]).reshape((4, 1))
        delta_t = from_state.time_to(to_state)
        q_B_G_prev = from_state.q_B_G

        k1 = 0.5 * Quaternion.big_omega_matrix(gyro_est_prev).dot(q_0)
        k2 = 0.5 * Quaternion.big_omega_matrix(gyro_est_mid).dot(q_0 + delta_t/2.0 * k1)
        k3 = 0.5 * Quaternion.big_omega_matrix(gyro_est_mid).dot(q_0 + delta_t/2.0 * k2)
        k4 = 0.5 * Quaternion.big_omega_matrix(gyro_est).dot(q_0 + delta_t*k3)

        q_perturb_vec = q_0 + delta_t/6.0 * (k1 + 2*k2 + 2*k3 + k4)
        return np.quaternion(*q_perturb_vec).normalized()
        
    @staticmethod
    def _propagate_accel(from_state, to_state, q_Bnext_Bcurrent):
        global_gravity = BodyState.global_gravity
        r_l1_l = Quaternion.to_rotation_matrix(q_Bnext_Bcurrent).T
        delta_t = from_state.time_to(to_state)
        s_hat = delta_t/2.0 * (r_l1_l.dot(to_state.acceleration_estimate) + from_state.acceleration_estimate)
        y_hat = delta_t/2.0 * s_hat
        r_prev_to_global = Quaternion.to_rotation_matrix(from_state.q_B_G).T
        v_delta = r_prev_to_global.dot(s_hat) - global_gravity*delta_t # !!! subtract gravity, not add !!!
        p_delta = from_state.v_B_G*delta_t + r_prev_to_global.dot(y_hat) - 0.5*global_gravity*(delta_t*delta_t) # !!! subtract gravity, not add !!!
        return p_delta, v_delta

    
class State(object):
    time = property(lambda self: self.body_state.time)
    q_B_G = property(lambda self: self.body_state.q_B_G)
    p_B_G = property(lambda self: self.body_state.p_B_G)
    v_B_G = property(lambda self: self.body_state.v_B_G)
    pose = property(lambda self: PoseGet(self))

    def __init__(self, accel_item, gyro_item, initial_accel_mean):
        assert isinstance(accel_item, ImuItem), 'Expected `accel_item` to be of type `ImuItem`. Got `{}`.'.format(type(accel_item))
        assert isinstance(gyro_item, ImuItem), 'Expected `gyro_item` to be of type `ImuItem`. Got `{}`.'.format(type(gyro_item))
        assert accel_item.time == gyro_item.time, "Gyroscope and accelerometer times do not match"
        
        self.b_g = np.zeros((3, 1))
        self.b_a = np.zeros((3, 1))
        self.T_g = np.eye(3)
        self.T_s = np.zeros((3, 3))
        self.T_a = np.eye(3)
        self.p_B_C = np.zeros((3, 1))
        self.f_x, self.f_y = 0, 0
        self.o_x, self.o_y = 0, 0
        self.k_1, self.k_2, self.k_3 = 0, 0, 0
        self.t_1, self.t_2 = 0, 0
        self.t_d, self.t_r = 0, 0

        acceleration_estimate = self.compute_acceleration_estimate(accel_item.value)
        rotation_estimate = self.compute_rotation_estimate(gyro_item.value, accel_item.value)
        q_B_G = self.estimate_initial_attitude(initial_accel_mean)
        p_B_G = np.zeros((3, 1))
        v_B_G = np.zeros((3, 1))
        self.body_state = BodyState.initialize(accel_item.time, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G)

        self.sigma = None

        self.camera_poses = []
        self.features_tracked_in_image = { }
        self.first_camera_pose_id = 0
        self.past_last_camera_pose_id = 0

    @staticmethod
    def get_body_to_camera_frame_rotation():
        "This is parameter and filter does not estimate it !!!"
        return np.quaternion(0.5, -0.5, 0.5, -0.5).conjugate()

    def propagate(self, accel_item, gyro_item):
        assert abs(accel_item.time - gyro_item.time) < 1e-6, 'Not at the same time. Gyro: {:.5f} accel: {:.5f}'.format(gyro_item.time, accel_item.time)
        acceleration_estimate = self.compute_acceleration_estimate(accel_item.value)
        rotation_estimate = self.compute_rotation_estimate(gyro_item.value, accel_item.value)
        self.body_state = BodyState.propagate(self.body_state, accel_item.time, rotation_estimate, acceleration_estimate)
        print_yellow('Propagation [{:.2f}, {:.2f}, {:.2f}]'.format(self.body_state.p_B_G[0, 0], self.body_state.p_B_G[1, 0], self.body_state.p_B_G[2, 0]))

    def initial_attitude_to_empty_state(self, initial_accel_mean):
        initial_attitude = self.estimate_initial_attitude(initial_accel_mean)
        self.state[0:4] = initial_attitude.components.reshape((4, 1))

    def estimate_initial_attitude(self, initial_accel_mean):
        #assert initial_accel_mean.shape == (3, 1), "Wrong `initial_accel_mean` shape {} expected (3, 1)".format(initial_accel_mean.shape)
        #gravity_body_est = self.compute_acceleration_estimate(initial_accel_mean)
        #gravity_global = BodyState.global_gravity
        #w = gravity_global.dot(gravity_body_est) + gravity_global[3, 0]*gravity_global[3, 0]
        #vec = np.cross(gravity_global.reshape((3, )), gravity_body_est.reshape((3, )))
        #return np.quaternion(w, *vec).normalized().conjugate()
        return np.quaternion(1, 0, 0, 0)

    def augment(self):
        assert self.first_camera_pose_id <= self.past_last_camera_pose_id, "Invalid camera pose ID"
        
        # Augment state vector
        camera_pose = BodyState.copy(self.body_state)
        self.camera_poses.append(camera_pose)

        # Augment Sigma matrix
        n = len(self.camera_poses) - 1
        J_pi = np.eye(9, 56+(9*n))
        sigma = np.zeros((56+9*(n+1), 56+9*(n+1)))
        sigma[0:(56+9*n), 0:(56+9*n)] = self.sigma
        sigma[0:(56+9*n), (56+9*n):] = self.sigma.dot(J_pi.T)
        sigma[(56+9*n):, 0:(56+9*n)] = J_pi.dot(self.sigma)
        sigma[(56+9*n):, (56+9*n):] = J_pi.dot(self.sigma).dot(J_pi.T)
        self.sigma = sigma

        self.past_last_camera_pose_id += 1
        assert self.first_camera_pose_id <= self.past_last_camera_pose_id, "Invalid camera pose ID"

    def remove_n_cam_poses(self, n):
        "Remove last $n$ camera poses"
        assert n >= 0, "$n$ has to be non negative"
        self.camera_poses = self.camera_poses[n:]

        m = len(self.camera_poses) + n
        sigma = np.zeros((56 + 9*(m-n), 56 + 9*(m-n)))
        sigma[0:56, 0:56] = self.sigma[0:56, 0:56]
        sigma[0:56, 56:] = self.sigma[0:56, (56+9*n):]
        sigma[56:, 0:56] = self.sigma[(56+9*n):, 0:56]
        sigma[56:, 56:] = self.sigma[(56+9*n):, (56+9*n):]
        self.sigma = sigma

        self.first_camera_pose_id += n
        assert self.first_camera_pose_id <= self.past_last_camera_pose_id, "Invalid camera pose ID {} - {}".format(self.first_camera_pose_id, self.past_last_camera_pose_id)

    def get_pose_to_pose_rotation(self, from_camera_pose_id, to_camera_pose_id):
        "Cache this !!!"
        i, j = from_camera_pose_id, to_camera_pose_id
        q_Cj_G = self.get_rotation_from_global_to_camera_pose(j)
        q_Ci_G = self.get_rotation_from_global_to_camera_pose(i)
        q_Cj_Ci = q_Cj_G * q_Ci_G.conjugate()
        return q_Cj_Ci

    def get_position_of_pose_in_another_pose(self, target_pose_id, reference_pose_id):
        "Cache this !!!"
        i, j = target_pose_id, reference_pose_id
        q_Cj_G = self.get_rotation_from_global_to_camera_pose(j)
        R_Cj_G = Quaternion.to_rotation_matrix(q_Cj_G)
        p_Ci_G = self.get_position_of_pose_in_global_frame(i)
        p_Cj_G = self.get_position_of_pose_in_global_frame(j)
        return R_Cj_G.dot(p_Ci_G - p_Cj_G)

    def get_rotation_from_global_to_camera_pose(self, camera_pose_id):
        "Cache this !!!"
        q_C_B = self.get_body_to_camera_frame_rotation()
        q_Bto_G = self.pose[camera_pose_id].q_B_G
        return q_C_B * q_Bto_G

    def get_position_of_pose_in_global_frame(self, camera_pose_id):
        "Cache this !!!"
        p_B_C = self.p_B_C

        q_C_B = self.get_body_to_camera_frame_rotation()
        R_B_C = Quaternion.to_rotation_matrix(q_C_B.conjugate())

        q_Bi_G = self.pose[camera_pose_id].q_B_G
        R_G_Bi = Quaternion.to_rotation_matrix(q_Bi_G.conjugate())

        p_Bi_G = self.pose[camera_pose_id].p_B_G

        return p_Bi_G - R_G_Bi.dot(R_B_C).dot(p_B_C)

    def time_to(self, other):
        "Get time delta between current and the other state"
        assert self.time < other.time, "Self should be before other"
        return other.time - self.time

    def compute_rotation_estimate(self, gyro, accel):
        return inv(self.T_g).dot(gyro - self.T_s.dot(accel) - self.b_g)

    def compute_acceleration_estimate(self, accel):
        return inv(self.T_a).dot(accel - self.b_a)


class FeatureTrack():
    def __init__(self, first_image, x, y):
        self.first_image = first_image
        self.last_image = first_image
        self.pixel_coordinates = [(x, y)]

    def next(self, x, y):
        self.last_image += 1
        self.pixel_coordinates.append((x, y))

    def track_length(self):
        return self.last_image - self.first_image + 1

    def get_pixel_coordinates_in_image(self, image_id):
        assert image_id >= self.first_image, "Requested image {} is before feature track which begins at {}".format(image_id, self.first_image)
        assert image_id <= self.last_image, "Requested image {} is after feature track which ends at {}".format(image_id, self.last_image)
        return np.asarray(self.pixel_coordinates[image_id - self.first_image]).reshape((2, 1))


class ImageTracker():
    def __init__(self):
        self.image_counter = -1
        self.last_image = None
        self.last_kps = None
        self.last_des = None
        self.last_image_was_published = False
        self.last_matches_image = None
        self.last_features = []
        self.orb = cv2.ORB(nfeatures=40)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    def track_next(self, image):
        self.image_counter += 1
        kps, des = self.orb.detectAndCompute(image, None)
        print(len(kps))
        assert kps is not None
        assert des is not None

        if self.last_image is None:
            self.last_kps = kps
            self.last_des = des
            self.last_image = image
            self.last_image_was_published = False
            for kp in kps:
                self.last_features.append(FeatureTrack(self.image_counter, kp.pt[0], kp.pt[1]))
            return []

        assert self.last_des.shape[1] == des.shape[1]

        matches = self.matcher.match(self.last_des, des)
        image = cv2.drawKeypoints(image, kps, color=(0, 255, 0), flags=0)

        matched_prev = [False] * len(self.last_kps)

        features = [None] * len(kps) 

        for match in matches:
            prev_idx = match.queryIdx
            curr_idx = match.trainIdx
            matched_prev[prev_idx] = True

            # Matched feature
            feature = self.last_features[prev_idx]

            kp = kps[curr_idx]
            feature.next(kp.pt[0], kp.pt[1])
            features[curr_idx] = feature

        # Features to residualize
        to_rezidualize = []
        for feature, matched in zip(self.last_features, matched_prev):
            if matched:
                continue
            to_rezidualize.append(feature)
        
        new_features = 0
        for i, feature in enumerate(features):
            if feature is not None:
                continue
            new_features += 1
            features[i] = FeatureTrack(self.image_counter, kps[i].pt[0], kps[i].pt[1])

        print("Features {}, new {}, matched {}, to rezidualize {}".format(len(kps), new_features, len(matches), len(to_rezidualize)))

        self.last_kps = kps
        self.last_des = des
        self.last_image = image
        self.last_image_was_published = False
        self.last_features = features

        return to_rezidualize

    def extract_old_features(self, age):
        mask = [feature.track_length() < age for feature in self.last_features]
        to_residualize = [f for f, m in zip(self.last_features, mask) if not m]
        self.last_features = [f for f, m in zip(self.last_features, mask) if m]
        self.last_kps = [kp for kp, m in zip(self.last_kps, mask) if m]
        self.last_des = self.last_des[np.asarray(mask), :]
        return to_residualize

    def image_to_publish(self):
        if (self.last_image is None) or self.last_image_was_published:
            return None
        else:
            self.last_image_was_published = True
            return self.last_image
        

class Msckf:
    def __init__(self):
        self.imu_subscriber = None
        self.camera_subscriber = None
        self.pointcloud_publisher = None
        self.br = None
        self.lock = threading.Lock()
        self.listener = None

        self.cam_to_process = None
        self.state = None
        self.image_tracker = ImageTracker()
        self.bridge = CvBridge()
        self.cam_poses = { }
        self.buffer_size_to_initialize = 50
        self.image_rows = None
        self.buffers = {
            ImuDevice.accelerometer: ImuBuffer(ImuDevice.accelerometer),
            ImuDevice.gyroscope: ImuBuffer(ImuDevice.gyroscope)
        }

    def callback_imu(self, data):
        print('got imu')
        self.lock.acquire()
        try:
            gyro = data.angular_velocity
            accel = data.linear_acceleration
            got_some_data = False
            if not (gyro.x == 0 and gyro.y == 0 and gyro.z == 0):
                self.process_imu(ImuDevice.gyroscope, data.header.stamp.to_sec(), gyro.x, gyro.y, gyro.z)
                got_some_data = True
            if not (accel.y == 0 and accel.y == 0 and accel.z == 0):
                self.process_imu(ImuDevice.accelerometer, data.header.stamp.to_sec(), accel.x, accel.y, accel.z)
                got_some_data = True
            if got_some_data:
                if self.state is None:
                    self.try_initialize()
                if self.state is not None:
                    updated = self.try_steps()
                    # Publish localization results
                    if updated:
                        self.publish_localization()
                        self.publish_image_if_new_available()
        except Exception as e:
            traceback.print_exc(file=sys.stderr)
            rospy.signal_shutdown("ROSPy Shutdown")
        finally:
            self.lock.release()

    def process_imu(self, device, timestamp, x, y, z):
        item = ImuItem(device, timestamp, np.asarray([x, y, z]).reshape((3, 1)))
        self.buffers[device].push(item)

    def callback_camera(self, data):
        print('got camera')
        self.lock.acquire()
        try:
            if self.state is None:
                return
            time = data.header.stamp.to_sec()
            if self.cam_to_process is not None:
                return
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
                sys.exit()
            
            assert cv_image.shape == (480, 640, 3)
            self.image_rows = 480

            self.cam_to_process = CameraItem(time, cv_image)
        except Exception as e:
            traceback.print_exc(file=sys.stderr)
            rospy.signal_shutdown("ROSPy Shutdown")
        finally:
            self.lock.release()

    def try_initialize(self):
        # Wait for forst $n$ measurements from both sensors and then initialize filter using them
        # I also use them for propagation to help filter initialize early (instead of waiting for more measurements)
        if min(len(self.buffers[ImuDevice.accelerometer]), len(self.buffers[ImuDevice.gyroscope])) < self.buffer_size_to_initialize:
            return

        accel_buffer = self.buffers[ImuDevice.accelerometer]
        gyro_buffer = self.buffers[ImuDevice.gyroscope]
        time = max(accel_buffer.min.time, gyro_buffer.min.time)
        accel_item = accel_buffer.interpolate(time)
        gyro_item = gyro_buffer.interpolate(time)
        if accel_buffer[1].time < gyro_buffer[1].time:
            accel_buffer.pop_min()
        else:
            gyro_buffer.pop_min()

        with file('config.yaml', 'r') as o:
            config = yaml.load(o)

        self.state = State(accel_item, gyro_item, accel_buffer.mean_value)
        self.state.body_state.q_B_G = np.quaternion(*config['orientation']['init'])
        self.state.body_state.p_B_G = np.asarray(config['position']['init']).reshape((3, 1))
        self.state.body_state.v_B_G = np.asarray(config['velocity']['init']).reshape((3, 1))
        self.state.b_a = np.asarray(config['accelerometer_bias']['init']).reshape((3, 1))
        self.state.b_g = np.asarray(config['gyroscope_bias']['init']).reshape((3, 1))
        self.state.T_g = np.asarray(config['gyroscope_shape']['init']).reshape((3, 3))
        self.state.T_s = np.asarray(config['gyroscope_acceleration_sensitivity']['init']).reshape((3, 3))
        self.state.T_a = np.asarray(config['accelerometer_shape']['init']).reshape((3, 3))
        self.state.p_B_C = np.asarray(config['body_position_in_camera_frame']['init']).reshape((3, 1))
        self.state.f_x, self.state.f_y = tuple(config['focal_length']['init'])
        self.state.o_x, self.state.o_y = tuple(config['optical_center']['init'])
        self.state.k_1, self.state.k_2, self.state.k_3 = tuple(config['radial_distortion']['init'])
        self.state.t_1, self.state.t_2 = tuple(config['tangential_distortion']['init'])
        self.t_d = config['camera_delay_time']['init']
        self.t_r = config['camera_readout_time']['init']

        sigma_diag = np.zeros(56)
        sigma_diag[0:3] = np.asarray(config['orientation']['sigma']).reshape((3, ))
        sigma_diag[3:6] = np.asarray(config['position']['sigma']).reshape((3, ))
        sigma_diag[6:9] = np.asarray(config['velocity']['sigma']).reshape((3, ))
        sigma_diag[9:12] = np.asarray(config['accelerometer_bias']['sigma']).reshape((3, ))
        sigma_diag[12:15] = np.asarray(config['gyroscope_bias']['sigma']).reshape((3, ))
        sigma_diag[15:24] = np.asarray(config['gyroscope_shape']['sigma']).reshape((9, ))
        sigma_diag[24:33] = np.asarray(config['gyroscope_acceleration_sensitivity']['sigma']).reshape((9, ))
        sigma_diag[33:42] = np.asarray(config['accelerometer_shape']['sigma']).reshape((9, ))
        sigma_diag[42:45] = np.asarray(config['body_position_in_camera_frame']['sigma']).reshape((3, ))
        sigma_diag[45:47] = np.asarray(config['focal_length']['sigma']).reshape((2, ))
        sigma_diag[47:49] = np.asarray(config['optical_center']['sigma']).reshape((2, ))
        sigma_diag[49:52] = np.asarray(config['radial_distortion']['sigma']).reshape((3, ))
        sigma_diag[52:54] = np.asarray(config['tangential_distortion']['sigma']).reshape((2, ))
        sigma_diag[54:55] = np.asarray(config['camera_delay_time']['sigma']).reshape((1, ))
        sigma_diag[55:56] = np.asarray(config['camera_readout_time']['sigma']).reshape((1, ))
        self.state.sigma = np.diag(sigma_diag)

        print("\033[0;32mFilter was successfully initialized at time {:.5f}\033[0m".format(self.state.time))

    def try_steps(self):
        assert self.state is not None
        if self.cam_to_process is None:
            return False
        camera_delay_time = self.state.t_d
        camera_readout_time = self.state.t_r
        # `camera_readout_time` might be negative
        camera_read_start = self.cam_to_process.time + camera_delay_time - abs(camera_readout_time) / 2
        camera_read_end = self.cam_to_process.time + camera_delay_time + abs(camera_readout_time) / 2

        has_accel_before_image_read = self.buffers[ImuDevice.accelerometer].min.time <= camera_read_start
        has_accel_after_image_read = self.buffers[ImuDevice.accelerometer].max.time >= camera_read_end
        has_gyro_before_image_read = self.buffers[ImuDevice.gyroscope].min.time <= camera_read_start
        has_gyro_after_image_read = self.buffers[ImuDevice.gyroscope].max.time >= camera_read_end
        accel_ok = has_accel_before_image_read and has_accel_after_image_read
        gyro_ok = has_gyro_before_image_read and has_gyro_after_image_read
        if (not accel_ok) or (not gyro_ok):
            return False
        # ready for processing
        self.propagate_up_to_time_incl(self.cam_to_process.time + camera_delay_time)
        print('Filter at time {}'.format(self.state.time))
        self.update()
        return True

    def propagate_up_to_time_incl(self, t):
        "Propatage up to time $t$. It is guaranteed that IMU measurements are available."
        assert isinstance(t, float), 'Expected `float`. Got {}.'.format(type(t))
        
        accel_buffer = self.buffers[ImuDevice.accelerometer]
        gyro_buffer = self.buffers[ImuDevice.gyroscope]

        should_continue = True
        while should_continue:
            accel_ptr = bisect.bisect_right(accel_buffer, self.state.time)
            gyro_ptr = bisect.bisect_right(gyro_buffer, self.state.time)

            step_time = min(accel_buffer[accel_ptr].time, gyro_buffer[gyro_ptr].time)

            if step_time > t:
                should_continue = False
                step_time = t
            accel_item = accel_buffer.interpolate(step_time)
            gyro_item = gyro_buffer.interpolate(step_time)
            self.state.propagate(accel_item, gyro_item)
        if self.state.time != t:
            print('Filter is propagated only to time {:.5f}. Expected {:.5f}'.format(self.state.time, t), sys.stderr)
            sys.exit()
        
    def update(self):
        'Perform MSCKF update step. State is already propagated to time $t + \\hat{t}_d$.'
        assert self.cam_to_process is not None
        assert abs(self.state.time - (self.cam_to_process.time + self.state.t_d)) < 1e-6, 'Filter is not propagated to {:.5f}. Current time {:.5f}'.format(self.cam_to_process.time, self.state.time)

        # Pose difference
        if len(self.state.camera_poses) > 0:
            dist = np.power(np.power(self.state.body_state.p_B_G - self.state.camera_poses[-1].p_B_G, 2).sum(), 0.5)
            if dist < 0.2:
                print_yellow('Distance is {:.3f}, skipping.'.format(dist))
                self.cam_to_process = None
                return

        image = self.cam_to_process.image
        self.cam_to_process = None

        try:
            t = self.tfBuffer.lookup_transform('world', 'base_link', rospy.Time.from_sec(self.state.time))
            pos = t.transform.translation
            att = t.transform.rotation
            print('Transform lookup OK')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print('Transformation lookup failed {}'.format(e))
            return

        to_rezidualize = self.image_tracker.track_next(image)
        too_old = self.image_tracker.extract_old_features(15)
        for feature in too_old:
            feature.pixel_coordinates = feature.pixel_coordinates[:-1]
            feature.last_image -= 1
            to_rezidualize.append(feature)
        self.cam_poses[self.image_tracker.image_counter] = len(self.image_tracker.last_kps)
        self.state.augment()

        self.state.camera_poses[-1].p_B_G = np.asarray([pos.x, pos.y, pos.z]).reshape((3, 1))
        self.state.camera_poses[-1].q_B_G = np.quaternion(att.w, att.x, att.y, att.z)
        
        print('Cam poses: {}'.format(len(self.state.camera_poses)))

        pointcloud = PointCloud()
        pointcloud.header = Header()
        pointcloud.header.stamp = rospy.Time.now()
        pointcloud.header.frame_id = 'world'

        # Do actual MSCKF update step
        H_list, r_list = [], []
        poses_to_remove = 0
        for feature in to_rezidualize:
            for i in range(feature.first_image, feature.last_image + 1):
                self.cam_poses[i] -= 1
                if self.cam_poses[i] == 0:
                    poses_to_remove += 1

            if feature.track_length() <= 3:
                continue
            
            global_position = self.compute_global_feature_estimate(self.state, feature)
            if global_position is None:
                print('Skipping feature due to unsuccessful global localization')
                continue
            poses_in_state = len(self.state.camera_poses)

            r_j_list, H_x_list, H_f_list = [], [], []

            for pose_id in range(feature.first_image, feature.last_image + 1):
                track_length = feature.track_length()
                j = poses_in_state-track_length-1+(pose_id-feature.first_image)
                x_z, y_z = feature.pixel_coordinates[pose_id - feature.first_image]
                z = np.asarray([x_z, y_z]).reshape((2, 1))

                camera_pose = self.state.camera_poses[j]
                N = self.image_rows / 2.0
                k = y_z - N
                t_r = self.state.t_r
                feature_time = camera_pose.time + k*t_r / N
                gyro_item = self.buffers[ImuDevice.gyroscope].interpolate(feature_time)
                accel_item = self.buffers[ImuDevice.accelerometer].interpolate(feature_time)
                rotation_estimate = self.state.compute_rotation_estimate(gyro_item.value, accel_item.value)
                acceleration_estimate = self.state.compute_acceleration_estimate(accel_item.value)
                feature_pose = BodyState.propagate(camera_pose, feature_time, rotation_estimate, acceleration_estimate)
                
                q_C_B = self.state.get_body_to_camera_frame_rotation()
                R_C_B = Quaternion.to_rotation_matrix(q_C_B)
                R_Bj_G = Quaternion.to_rotation_matrix(feature_pose.q_B_G)

                p_B_C = self.state.p_B_C
                rotation_compound = Quaternion.to_rotation_matrix(q_C_B * feature_pose.q_B_G)
                p_f_C = rotation_compound.dot(global_position - feature_pose.p_B_G) + p_B_C
                z_hat = self.pinhole_model(self.state, p_f_C)
                assert z_hat.shape == (2, 1)

                r_j = z - z_hat
                assert r_j.shape == (2, 1)

                J_h = self.camera_jacobian(p_f_C, self.state)
                M_i_j = J_h.dot(rotation_compound)

                H_x_Bj = M_i_j.dot(np.hstack([
                    Quaternion.cross_matrix(global_position - feature_pose.p_B_G),
                    -1*np.eye(3),
                    -1*(k*t_r / N) * np.eye(3) 
                ]))
                H_f_j = M_i_j
                assert H_f_j.shape == (2, 3)

                z_by_p_B_C = J_h
                assert z_by_p_B_C.shape == (2, 3)

                u = global_position[0, 0] / global_position[2, 0]
                v = global_position[1, 0] / global_position[2, 0]
                r = u*u + v*v
                d_r = 1 + self.state.k_1*r + self.state.k_2*(r*r) + self.state.k_3*(r**3)
                d_t = np.array([
                    2*u*v*self.state.t_1 + (r + 2*(u*u))*self.state.t_2,
                    2*u*v*self.state.t_2 + (r + 2*(v*v))*self.state.t_1
                ]).reshape((2, 1))
                h_by_fx = np.array([d_r*u + d_t[0, 0], 0]).reshape((2, 1))
                h_by_fy = np.array([0, d_r*v + d_t[1, 0]]).reshape((2, 1))
                h_by_ox = np.array([1.0, 0.0]).reshape((2, 1))
                h_by_oy = np.array([0.0, 1.0]).reshape((2, 1))
                h_by_k1 = np.array([
                    self.state.f_x * u * r,
                    self.state.f_y * v * r
                ]).reshape((2, 1))
                h_by_k2 = r * h_by_k1
                h_by_k3 = r * h_by_k2
                h_by_t1 = np.array([
                    self.state.f_x * (2*u*v),
                    self.state.f_y * (r + 2*(v*v))
                ]).reshape((2, 1))
                h_by_t2 = np.array([
                    self.state.f_x * (r + 2*(u*u)),
                    self.state.f_y * (2*u*v)
                ]).reshape((2, 1))

                z_by_x_cam = np.hstack([
                    h_by_fx,
                    h_by_fy,
                    h_by_ox,
                    h_by_oy,
                    h_by_k1,
                    h_by_k2,
                    h_by_k3,
                    h_by_t1,
                    h_by_t2
                ])
                assert z_by_x_cam.shape == (2, 9)
                
                cross_matrix_part = Quaternion.cross_matrix(R_Bj_G.dot(global_position - feature_pose.p_B_G))
                z_by_t_d = J_h.dot(R_C_B).dot(cross_matrix_part.dot(feature_pose.rotation_estimate) - R_Bj_G.dot(feature_pose.v_B_G))
                z_by_t_r = (k/N)*z_by_t_d

                H_c_j = np.hstack([z_by_p_B_C, z_by_x_cam, z_by_t_d, z_by_t_r])
                assert H_c_j.shape == (2, 14), 'Expected `H_c_j` to have shape (2, 14). Got {}'.format(H_c_j.shape)
                
                H_x_j = np.zeros((2, 56 + 9*len(self.state.camera_poses)))
                H_x_j[:, 42:56] = H_c_j
                H_x_j[:, (56+9*j):(56+9*(j+1))] = H_x_Bj

                r_j_list.append(r_j)
                H_x_list.append(H_x_j)
                H_f_list.append(H_f_j)
                
            r = np.vstack(r_j_list)
            H_x = np.vstack(H_x_list)
            H_f = np.vstack(H_f_list)

            #q, _ = np.linalg.qr(H_f, mode='complete')
            #A = q[:, -1*(H_f.shape[0]-H_f.shape[1]):]
            A = null(H_f.T, eps=1e-18)
            #np.testing.assert_allclose(A.T.dot(H_f), np.zeros((rank, H_f.shape[1])))
            #np.testing.assert_allclose(A.T.dot(H_f), np.zeros((H_f.shape[0] - H_f.shape[1], H_f.shape[1])), atol=1e-4, rtol=0)
            assert A.shape == (H_f.shape[0], H_f.shape[0]-H_f.shape[1]), 'Expected `A` to have shape {}. Got {}'.format((H_f.shape[0], H_f.shape[0]-H_f.shape[1]), A.shape)
            np.testing.assert_allclose(H_f.T.dot(A), np.zeros((H_f.shape[1], H_f.shape[0] - H_f.shape[1])), atol=1e-4, rtol=0)
            np.testing.assert_allclose(A.T.dot(H_f), np.zeros((H_f.shape[0] - H_f.shape[1], H_f.shape[1])), atol=1e-4, rtol=0)
            np.testing.assert_allclose(A.T.dot(A), np.eye(H_f.shape[0]-H_f.shape[1]), atol=1e-4, rtol=0)

            r_0 = A.T.dot(r)
            H_0 = A.T.dot(H_x)

            try:
                gamma_inner = inv(H_0.dot(self.state.sigma).dot(H_0.T))
            except np.linalg.LinAlgError as e:
                print(e)
                continue
            gamma_i = r_0.T.dot(gamma_inner).dot(r_0)
            cdf_threshold = scipy.stats.chi2.cdf(0.95, r_0.shape[0])
            # if gamma_i[0, 0] > cdf_threshold:
            #     print('Feature rejected by gating test. Gamma: {:.5f} threshold: {:.5f}'.format(gamma_i[0, 0], cdf_threshold))
            #     continue

            pointcloud.points.append(Point32(global_position[0, 0], global_position[1, 0], global_position[2, 0]))

            r_list.append(r_0)
            H_list.append(H_0)

        self.pointcloud_publisher.publish(pointcloud)

        if len(r_list) == 0:
            print('\033[0;35mNo features to update with.\033[0m')
            self.state.remove_n_cam_poses(poses_to_remove)
            return

        r = np.vstack(r_list)
        H = np.vstack(H_list)

        print('Passed {} out of {}'.format(len(r_list), len(to_rezidualize)))

        sigma_im = 1e-2
        q, r_qr = np.linalg.qr(H, mode='complete')
        assert q.shape == (H.shape[0], H.shape[0])
        num_all_zero_rows = np.all(r_qr == 0, axis=1).sum()
        q1 = q[:, 0:(r_qr.shape[0]-num_all_zero_rows)]
        T_H = r_qr[0:(r_qr.shape[0]-num_all_zero_rows), :]
        assert q1.shape[1] == T_H.shape[0], 'q1.shape[1] == {} & T_H.shape[0] == {}'.format(q1.shape[1], T_H.shape[0])

        R_q = sigma_im * np.eye(q1.shape[1])
        K = self.state.sigma.dot(T_H.T).dot(inv(T_H.dot(self.state.sigma).dot(T_H.T) + R_q))

        I_beta = np.eye(self.state.sigma.shape[0])
        sigma_part = I_beta - K.dot(T_H)
        self.state.sigma = sigma_part.dot(self.state.sigma).dot(sigma_part.T) + K.dot(R_q).dot(K.T)
        r_q = q1.T.dot(r)
        delta_x = K.dot(r_q)

        # Update State
        #self.state.body_state.q_B_G = self.state.q_B_G * np.quaternion(1.0, *((0.5*delta_x[0:3, 0]).tolist())).normalized()
        #self.state.body_state.p_B_G += delta_x[3:6, 0].reshape((3, 1))
        #self.state.body_state.v_B_G += delta_x[6:9, 0].reshape((3, 1))
        #self.state.b_g += delta_x[9:12, 0].reshape((3, 1))
        #self.state.b_a += delta_x[12:15, 0].reshape((3, 1))
        #self.state.T_g += delta_x[15:24, 0].reshape((3, 3))
        #self.state.T_s += delta_x[24:33, 0].reshape((3, 3))
        #self.state.T_a += delta_x[33:42, 0].reshape((3, 3))
        #self.state.p_B_C += delta_x[42:45, 0].reshape((3, 1))
        #self.state.f_x += delta_x[45, 0]
        #self.state.f_y += delta_x[46, 0]
        #self.state.o_x += delta_x[47, 0]
        #self.state.o_y += delta_x[48, 0]
        #self.state.k_1 += delta_x[49, 0]
        #self.state.k_2 += delta_x[50, 0]
        #self.state.k_3 += delta_x[51, 0]
        #self.state.t_1 += delta_x[52, 0]
        #self.state.t_2 += delta_x[53, 0]
        #self.state.t_d += delta_x[54, 0]
        #self.state.t_r += delta_x[55, 0]
        #for i in range(len(self.state.camera_poses)):
        #    cam_pose = self.state.camera_poses[i]
        #    cam_pose.q_B_G = cam_pose.q_B_G * np.quaternion(1, *((0.5*delta_x[(56+(9*i)+0):56+(9*i)+3, 0]).tolist())).normalized()
        #    cam_pose.p_B_G += delta_x[(56+(9*i)+3):56+(9*i)+6, 0].reshape((3, 1))
        #    cam_pose.v_B_G += delta_x[(56+(9*i)+6):56+(9*i)+9, 0].reshape((3, 1))

        print_yellow_bold('p: ({:.1f}, {:.1f}, {:.1f})'.format(self.state.p_B_G[0, 0], self.state.p_B_G[1, 0], self.state.p_B_G[2, 0]))
        print_yellow('bias accel: [{:.5f}, {:.5f}, {:.5f}]'.format(self.state.b_a[0, 0], self.state.b_a[1, 0], self.state.b_a[2, 0]))
        print_yellow('bias gyro: [{:.5f}, {:.5f}, {:.5f}]'.format(self.state.b_g[0, 0], self.state.b_g[1, 0], self.state.b_g[2, 0]))
        print_yellow('tangential dist.: ({:.5f}, {:.5f}, {:.5f})'.format(self.state.k_1, self.state.k_2, self.state.k_3))
        print_yellow('radial dist.: ({:.5f}, {:.5f})'.format(self.state.t_1, self.state.t_2))
        print_yellow('t_d: {:.5f}'.format(self.state.t_d))
        print_yellow('t_r: {:.5f}'.format(self.state.t_r))

        # Clean up unused camera poses
        self.state.remove_n_cam_poses(poses_to_remove)

    def triangulate(self, z1, z2, R_C1_C0, p_C1_C0):
        v1 = np.asarray([z1[0, 0], z1[1, 0], 1]).reshape((3, 1))
        v2 = np.asarray([z2[0, 0], z2[1, 0], 1]).reshape((3, 1))
        v1 = v1 / np.linalg.norm(v1)
        v2 = v2 / np.linalg.norm(v2)
        A = np.zeros((3, 2))
        A[:, 0:1] = v1
        A[:, 1:2] = -1*R_C1_C0.dot(v2)
        b = p_C1_C0
        x, residuals, rank, s = np.linalg.lstsq(a=A, b=b)
        return x[0] * v1

    def h(self, x, state):
        res = np.zeros((int(2*x.shape[0]/3), 1))
        res[0::2, :] = state.f_x * x[0::3, :] / x[2::3, :] + state.o_x
        res[1::2, :] = state.f_y * x[1::3, :] / x[2::3, :] + state.o_y
        return res

    def g(self, x, state, feature):
        alpha, beta, rho = x[0, 0], x[1, 0], x[2, 0]
        n = feature.track_length()
        res = np.zeros((3*n, 1))
        for i in range(n):
            R_Ci_C0 = Quaternion.to_rotation_matrix(state.get_pose_to_pose_rotation(feature.first_image, feature.first_image + i))
            p_C0_Ci = state.get_position_of_pose_in_another_pose(target_pose_id=(feature.first_image), reference_pose_id=feature.first_image + i)
            res[(3*i):(3*(i+1)), :] = R_Ci_C0.dot(np.asarray([alpha, beta, 1.0]).reshape((3, 1))) + rho * p_C0_Ci
        return res

    def feature_residual(self, x, state, feature):
        z_true = np.asarray(feature.pixel_coordinates).flatten()[:, np.newaxis]
        return (z_true - self.h(self.g(x.reshape((3, 1)), state, feature), state)).flatten()

    def camera_jacobian(self, p_f_C, state):
        h_lambda = lambda x: self.h(x[:, np.newaxis], state).flatten()
        return approx_jacobian(p_f_C.flatten(), h_lambda, epsilon=1e-6)

    def compute_global_feature_estimate(self, state, feature, max_steps=100, threshold=1e-12):
        feature_length = feature.track_length()
        # initial guess, this might be better

        R_C1_C0 = Quaternion.to_rotation_matrix(state.get_pose_to_pose_rotation(feature.first_image, feature.first_image + 1))
        p_C1_C0 = state.get_position_of_pose_in_another_pose(target_pose_id=(feature.first_image + 1), reference_pose_id=feature.first_image)
        z0 = np.asarray(feature.pixel_coordinates[0]).reshape((2, 1))
        z1 = np.asarray(feature.pixel_coordinates[1]).reshape((2, 1))
        z0[0, 0] = (z0[0, 0] - state.o_x) / state.f_x
        z0[1, 0] = (z0[1, 0] - state.o_y) / state.f_y
        z1[0, 0] = (z1[0, 0] - state.o_x) / state.f_x
        z1[1, 0] = (z1[1, 0] - state.o_y) / state.f_y
        p_f_init = self.triangulate(z0, z1, R_C1_C0, p_C1_C0)
        estimate = 1.0/p_f_init[2, 0] * p_f_init
        print(R_C1_C0)
        print(p_C1_C0)
        print(state.camera_poses[0].p_B_G)
        print(state.camera_poses[1].p_B_G)
        print(state.get_position_of_pose_in_global_frame(feature.first_image))
        print(state.get_position_of_pose_in_global_frame(feature.first_image + 1))
        print('p_f_init: {}'.format(p_f_init))
        
        # Solve
        x0 = estimate.flatten()
        print('Initial reprojection errors: ')
        print(self.feature_residual(x0, state=state, feature=feature))
        res = scipy.optimize.least_squares(self.feature_residual, x0, kwargs={'state': state, 'feature': feature})

        if not res.success:
            print('Unsuccessful')
            return None
        print(res)

        R_G_C0 = Quaternion.to_rotation_matrix(state.get_rotation_from_global_to_camera_pose(feature.first_image).conjugate())
        p_C0_G = state.get_position_of_pose_in_global_frame(feature.first_image)
        p_f_G_est = 1/res.x[2] * R_G_C0.dot(np.asarray([res.x[0], res.x[1], 1.0]).reshape((3, 1))) + p_C0_G

        residual = self.feature_residual(res.x, state=state, feature=feature)
        print('Reprojection errors: ')
        print(residual)

        # if res.x[2] < 0:
        #     raise Exception('Killing feature')

        return p_f_G_est
        

        # camera_derivatives = [None] * feature_length
        # for step in range(max_steps):
        #     jacobian = np.zeros((2*feature_length, 3))
        #     errors = np.zeros((2*feature_length, 1))

        #     for i in range(feature_length):
        #         to_this_rotation = Quaternion.to_rotation_matrix(state.get_pose_to_pose_rotation(feature.first_image, feature.first_image + i))
        #         to_this_position = state.get_position_of_pose_in_another_pose(target_pose_id=(feature.first_image), reference_pose_id=feature.first_image + i)

        #         estimate_in_frame = Quaternion.to_rotation_matrix(state.get_rotation_from_global_to_camera_pose(feature.first_image + i)).dot(estimate - state.get_position_of_pose_in_global_frame(feature.first_image + i))

        #         estimate_in_frame = to_this_rotation.dot(np.asarray([estimate[0, 0], estimate[1, 0], 1.0]).reshape((3, 1))) + estimate[2, 0]*to_this_position

        #         # if i == 0:
        #         #     print('Jeste vetsi vole')
        #         #     print(to_this_rotation)
        #         #     print(to_this_position)
        #         #     print(estimate)
        #         #     print(estimate_in_frame)
        #         #     print('konec')

        #         camera_derivative = self.calculate_camera_projection_estimate(state, estimate_in_frame)
        #         camera_derivatives[i] = camera_derivative

        #         # Calculate projection derivative $\dfrac{\partial g_i}{\partial \theta}$
        #         projection_derivative = np.zeros((3, 3))
        #         projection_derivative[:, 0:1] = to_this_rotation.dot(np.array([1.0, 0.0, 0.0]).reshape((3, 1)))
        #         projection_derivative[:, 1:2] = to_this_rotation.dot(np.array([0.0, 1.0, 0.0]).reshape((3, 1)))
        #         projection_derivative[:, 2:] = to_this_position

        #         # print('vole {}'.format(i))
        #         # print(self.pinhole_model(state, estimate_in_frame))
        #         # print(self.pinhole_model(state, estimate))
        #         error = feature.get_pixel_coordinates_in_image(feature.first_image + i) - self.pinhole_model(state, estimate_in_frame)

        #         lower, upper = 2*i, 2*(i+1)
        #         jacobian[lower:upper,:] = camera_derivative.dot(projection_derivative)
        #         errors[lower:upper,:] = error

        #     #print('Iteration {}'.format(step))
        #     #print(errors)

        #     if np.absolute(errors).max() < threshold:
        #         print("Breaking at iteration {} with errors: {}".format(step, np.absolute(errors).max()))
        #         break
        #     delta = inv(jacobian.T.dot(jacobian)).dot(jacobian.T).dot(errors)
        #     estimate = estimate + delta
        # print('Step: {}/{} ({:.5f})'.format(step, max_steps, np.absolute(errors).max()))

        # # TODO: Remove
        # p_f_init = self.triangulate(z0, z1, R_C1_C0, p_C1_C0)
        # estimate = 1.0/p_f_init[2, 0] * p_f_init

        # # Compute global position
        # rotation_from_first_to_global = Quaternion.to_rotation_matrix(state.get_rotation_from_global_to_camera_pose(feature.first_image)).T
        # position_of_first_in_global = state.get_position_of_pose_in_global_frame(feature.first_image)
        # alpha, beta, rho = estimate[0, 0], estimate[1, 0], estimate[2, 0]

        # global_position = 1.0/rho * rotation_from_first_to_global.dot(np.array([alpha, beta, 1.0]).reshape((3, 1))) + position_of_first_in_global
        
        # print('estimate:')
        # to_first_rotation = Quaternion.to_rotation_matrix(state.get_pose_to_pose_rotation(feature.first_image, feature.first_image))
        # to_first_position = state.get_position_of_pose_in_another_pose(target_pose_id=(feature.first_image), reference_pose_id=feature.first_image)
        # estimate_in_first = to_first_rotation.dot(np.asarray([estimate[0, 0], estimate[1, 0], 1.0]).reshape((3, 1))) + estimate[2, 0]*to_first_position
        # actual = feature.get_pixel_coordinates_in_image(feature.first_image)
        # estimated = self.pinhole_model(state, estimate_in_first)
        # print('Actual: [{:.0f}, {:.0f}]'.format(actual[0, 0], actual[1, 0]))
        # print('Projected: [{:.0f}, {:.0f}]'.format(estimated[0, 0], estimated[1, 0]))
        # print(rotation_from_first_to_global)
        # print(position_of_first_in_global)
        # print(estimate)
        # print(global_position)
        
        # return global_position, camera_derivatives

    def calculate_camera_projection_estimate(self, state, estimate):
        x, y, z = estimate[0, 0], estimate[1, 0], estimate[2, 0]
        #print('({:.2f}, {:.2f}, {:.2f})'.format(x, y, z))

        uv_by_xyz = np.asarray([y/(z*z), x/(z*z), -2*x*y/(z*z*z)]).reshape((1, 3))
        u_by_xyz = np.asarray([1.0/z, 0.0, -1*x/(z*z)]).reshape((1, 3))
        v_by_xyz = np.asarray([0.0, 1.0/z, -1*y/(z*z)]).reshape((1, 3))

        r_by_xyz = np.asarray([2*x/(z*z), 2*y/(z*z), -2.0/(z*z*z)*(x*x + y*y)]).reshape((1, 3))
        r_squared_by_xyz = np.asarray([4.0*x*(x*x + y*y)/np.power(z, 4), 4.0*y*(x*x + y*y)/np.power(z, 4), -4.0*np.power((x*x + y*y), 2)/np.power(z, 5)]).reshape((1, 3))
        r_cubed_by_xyz = np.asarray([
            6.0*x*np.power(x*x + y*y, 2)/np.power(z, 6),
            6.0*y*np.power(x*x + y*y, 2)/np.power(z, 6),
            -6.0*np.power(x*x + y*y, 3)/np.power(z, 7)
        ]).reshape((1, 3))

        u, v = x/z, y/z
        r = u*u + v*v
        k1, k2, k3 = state.k_1, state.k_2, state.k_3
        t1, t2 = state.t_1, state.t_2
        fx, fy = state.f_x, state.f_y
        d_r_by_xyz = 1 + k1*r_by_xyz + k2*r_squared_by_xyz + k3*r_cubed_by_xyz
        d_t_by_xyz = np.array([
            2*t1*uv_by_xyz + t2*r_by_xyz + 4.0*t2*u*u_by_xyz,
            2*t2*uv_by_xyz + t1*r_by_xyz + 4.0*t1*v*v_by_xyz
        ]).reshape((2, 3))

        d_r = 1 + k1*r + k2*np.power(r, 2) + k3*np.power(r, 3)
        #d_t = np.asarray([
        #    [2.0*u*v*t1 + (r + 2.0*u*u)*t2],
        #    [2.0*u*v*t2 + (r + 2.0*v*v)*t1]
        #]).reshape((2, 1))

        f = np.asarray([
            [fx, 0], 
            [0, fy]
        ]).reshape((2, 2))
        partial_derivatives = np.asarray([
            [d_r/z + d_r_by_xyz[0, 0]*u + d_t_by_xyz[0, 0], d_r_by_xyz[0, 1]*u + d_t_by_xyz[0, 1], -1.0*d_r/z*u + d_r_by_xyz[0, 2]*u + d_t_by_xyz[0, 2]],
            [d_r_by_xyz[0, 0]*v + d_t_by_xyz[1, 0], d_r/z + d_r_by_xyz[0, 1]*v + d_t_by_xyz[1, 1], -1.0*d_r/z*v + d_r_by_xyz[0, 2]*v + d_t_by_xyz[1, 2]]
        ]).reshape((2, 3))

        return f.dot(partial_derivatives)

    def pinhole_model(self, state, point_3d):
        "Point is $(X/Z, Y/Z, 1/Z)$"
        x, y, z = tuple(point_3d)
        u, v = x/z, y/z
        r = u*u + v*v
        ox, oy = state.o_x, state.o_y
        fx, fy = state.f_x, state.f_y
        k1, k2, k3 = state.k_1, state.k_2, state.k_3
        t1, t2 = state.t_1, state.t_2

        o = np.asarray([ox, oy]).reshape((2, 1))
        f = np.asarray([
            [fx, 0],
            [0, fy]
        ])
        d_r = 1 + k1*r + k2*np.power(r, 2) + k3*np.power(r, 3)
        d_t = np.asarray([
            2.0*u*v*t1 + (r + 2.0*u*u)*t2,
            2.0*u*v*t2 + (r + 2.0*u*v)*t1
        ])
        
        return o + f.dot(d_r * np.asarray([u, v]).reshape(2, 1) + d_t)

    def interpolate(self, earlier, later, to_time):
        earlier_time = earlier[0]
        later_time = later[0]
        assert earlier_time <= to_time, "Cannot extrapolate IMU data into the past"
        assert to_time <= later_time, "Cannot extrapolate IMU data into the future"
        t = (to_time - earlier_time) / (later_time - earlier_time)
        interpolated = t*earlier[1] + (1-t)*later[1]
        return (to_time, interpolated)

    def publish_localization(self):
        position = self.state.p_B_G
        attitude = self.state.q_B_G

        t = TransformStamped()
        t.header.stamp = rospy.Time.from_sec(self.state.time)
        t.header.frame_id = 'world'
        t.child_frame_id = 'msckf'
        t.transform.translation.x = position[0, 0]
        t.transform.translation.y = position[1, 0]
        t.transform.translation.z = position[2, 0]
        t.transform.rotation.x = attitude.x
        t.transform.rotation.y = attitude.y
        t.transform.rotation.z = attitude.z
        t.transform.rotation.w = attitude.w

        self.br.sendTransform(t)

    def publish_image_if_new_available(self):
        image = self.image_tracker.image_to_publish()
        if image is None:
            return
        msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.image_publisher.publish(msg)
        
    def run(self):
        rospy.init_node('msckf', anonymous=True)

        # Reset Gazebo model
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = 'filterbot'
        model_state.pose.position.x = 0
        model_state.pose.position.y = 0
        model_state.pose.position.z = 0
        model_state.pose.orientation.x = 0
        model_state.pose.orientation.y = 0
        model_state.pose.orientation.z = 0
        model_state.pose.orientation.w = 1
        set_model_state(model_state)

        #self.imu_subscriber = rospy.Subscriber("/iosmsg/imu_data", Imu, self.callback_imu)
        self.imu_subscriber = rospy.Subscriber("/filterbot/imu_data", Imu, self.callback_imu)
        #self.camera_subscriber = rospy.Subscriber("/image_topic_2", Image, self.callback_camera)
        self.camera_subscriber = rospy.Subscriber("/filterbot/camera/image_raw", Image, self.callback_camera)
        self.br = tf2_ros.TransformBroadcaster()
        self.image_publisher = rospy.Publisher("/msckf/image", Image, queue_size=10)
        self.pointcloud_publisher = rospy.Publisher('/msckf/pointcloud', PointCloud, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.spin()


def main():
    msckf = Msckf()
    msckf.run()

if __name__ == '__main__':
    main()