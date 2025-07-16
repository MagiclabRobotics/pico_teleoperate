
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6



def get_odd_data(data):
    pos = data[:3]
    if(len(data[3:])==4):
        eul = quat_to_eul(data[3:])
    else:
        eul = data[3:]
    rotation = eul_to_rot(eul)

    odd_matrix = np.zeros((4, 4))
    odd_matrix[3, 3] = 1
    odd_matrix[:3, :3] = rotation
    odd_matrix[:3, 3] = pos.transpose()
    return odd_matrix



def rot_to_eul(R_matrix, degrees=False):
    rotation = R.from_matrix(R_matrix)
    euler_angles = rotation.as_euler('xyz', degrees=degrees)  # 默认使用zyx顺序，如果需要度为单位
    return euler_angles

def rot_to_quat(rotation_matrix):
    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()
    return quaternion
def eul_to_rot(euler_angles):
    rotation = R.from_euler('xyz', euler_angles)
    rotation_matrix = rotation.as_matrix()
    return rotation_matrix

def eul_to_quat(euler_angles,degrees=False):
    rotation = R.from_euler('xyz', euler_angles, degrees=degrees)
    quaternion = rotation.as_quat()
    return quaternion

def quat_to_eul(q,degrees=False):
    rotation = R.from_quat(q)
    euler_angles = rotation.as_euler('xyz', degrees=degrees)
    return euler_angles

def quat_to_rot(q):
    r = R.from_quat(q)
    # 获取旋转矩阵
    rotation_matrix = r.as_matrix()
    return rotation_matrix

