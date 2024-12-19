import math
import numpy as np
from pyquaternion import Quaternion


# 欧拉角转四元数
def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return Quaternion(w, x, y, z)

# 四元数转欧拉角
def quaternion_to_euler(w, x, y, z):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

def euler_to_rotation_matrix(roll, pitch, yaw, ZYX=True):
    # 绕X轴旋转
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    # 绕Y轴旋转
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    # 绕Z轴旋转
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # 合并旋转矩阵
    if ZYX:
        R = np.dot(R_x, np.dot(R_y, R_z))
    else:
        R = np.dot(R_z, np.dot(R_y, R_x))
    
    return R

# 旋转矩阵转欧拉角
def rotation_matrix_to_euler(R, ZYX=False):
    if ZYX:
        pass
    else:
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0

    return roll, pitch, yaw

# 欧拉角转角度
def euler_to_degrees(roll, pitch, yaw):
    return roll * (180.0 / math.pi), pitch * (180.0 / math.pi), yaw * (180.0 / math.pi)

# 角度转欧拉角
def degrees_to_euler(roll_deg, pitch_deg, yaw_deg):
    return roll_deg * (math.pi / 180.0), pitch_deg * (math.pi / 180.0), yaw_deg * (math.pi / 180.0)

def quat2euler2quat():
    initial_quaternion = Quaternion(axis=[1, 0, 0], angle=np.pi/4)
    print("初始化四元数:", initial_quaternion)

    yaw, pitch, roll = initial_quaternion.yaw_pitch_roll

    # 步骤3：使用获取的欧拉角转换回四元数
    # 重新生成四元数，按照相同顺序应用欧拉角 (yaw -> pitch -> roll)
    reconstructed_quaternion = (
        Quaternion(axis=[0, 0, 1], angle=yaw) *  # 绕Z轴的旋转
        Quaternion(axis=[0, 1, 0], angle=pitch) *  # 绕Y轴的旋转
        Quaternion(axis=[1, 0, 0], angle=roll)  # 绕X轴的旋转
    )
    print("从欧拉角转换得到的四元数:", reconstructed_quaternion)

    # 步骤4：验证四元数是否一致
    # 比较初始的四元数与重新构建的四元数
    is_close = np.allclose(initial_quaternion.elements, reconstructed_quaternion.elements, atol=1e-6)
    print("四元数是否一致:", is_close)
    import ipdb; ipdb.set_trace()

def euler2quat2euler():
    yaw, pitch, roll = [np.pi/3, np.pi/4, np.pi/6]
    reconstructed_quaternion = (
        Quaternion(axis=[0, 0, 1], angle=yaw) *  # 绕Z轴的旋转
        Quaternion(axis=[0, 1, 0], angle=pitch) *  # 绕Y轴的旋转
        Quaternion(axis=[1, 0, 0], angle=roll)  # 绕X轴的旋转
    )
    yaw1, pitch1, roll1 = reconstructed_quaternion.yaw_pitch_roll
    import ipdb; ipdb.set_trace()

def quat2euler2romatrix():
    # 步骤1：初始化一个四元数
    # 假设我们选择通过轴-角表示生成四元数（例如绕Y轴旋转45度）
    initial_quaternion = Quaternion(axis=[0, 1, 0], angle=np.pi/4)
    print("初始化四元数:", initial_quaternion)

    # 步骤2：从四元数提取欧拉角 (yaw, pitch, roll)
    yaw, pitch, roll = initial_quaternion.yaw_pitch_roll
    print("提取的欧拉角 (yaw, pitch, roll):", yaw, pitch, roll)

    # 步骤3：获取旋转矩阵
    rotation_matrix_from_quaternion = initial_quaternion.rotation_matrix
    print("从四元数生成的旋转矩阵:\n", rotation_matrix_from_quaternion)

    # 步骤4：使用获取的欧拉角重构四元数
    reconstructed_quaternion_from_euler = (
        Quaternion(axis=[0, 0, 1], angle=yaw) *  # 绕Z轴的旋转
        Quaternion(axis=[0, 1, 0], angle=pitch) *  # 绕Y轴的旋转
        Quaternion(axis=[1, 0, 0], angle=roll)  # 绕X轴的旋转
    )
    print("从欧拉角重构的四元数:", reconstructed_quaternion_from_euler)

    # 步骤5：从欧拉角生成旋转矩阵
    rotation_matrix_from_euler = reconstructed_quaternion_from_euler.rotation_matrix
    print("从欧拉角生成的旋转矩阵:\n", rotation_matrix_from_euler)

    # 验证两个旋转矩阵是否一致
    rotation_matrices_close = np.allclose(rotation_matrix_from_quaternion, rotation_matrix_from_euler, atol=1e-6)
    print("旋转矩阵是否一致:", rotation_matrices_close)

    # 步骤6：从旋转矩阵转换为四元数
    quaternion_from_rotation_matrix = Quaternion(matrix=rotation_matrix_from_euler)
    print("从旋转矩阵重构的四元数:", quaternion_from_rotation_matrix)

    # 步骤7：验证四元数是否与初始值一致
    quaternions_close = np.allclose(initial_quaternion.elements, quaternion_from_rotation_matrix.elements, atol=1e-6)
    print("四元数是否一致:", quaternions_close)

    # 步骤8：从旋转矩阵转换为欧拉角
    yaw_from_matrix, pitch_from_matrix, roll_from_matrix = quaternion_from_rotation_matrix.yaw_pitch_roll
    print("从旋转矩阵提取的欧拉角 (yaw, pitch, roll):", yaw_from_matrix, pitch_from_matrix, roll_from_matrix)

    # 验证欧拉角是否与初始值一致
    euler_angles_close = np.allclose([yaw, pitch, roll], [yaw_from_matrix, pitch_from_matrix, roll_from_matrix], atol=1e-6)
    print("欧拉角是否一致:", euler_angles_close)
    import ipdb; ipdb.set_trace()

def test():

    # 欧拉角 [Yaw, Pitch, Roll] 定义
    yaw = np.pi / 3  # Yaw (绕Z轴旋转)
    pitch = np.pi / 4  # Pitch (绕Y轴旋转)
    roll = np.pi / 6  # Roll (绕X轴旋转)
    import ipdb; ipdb.set_trace()
    rot = euler_to_rotation_matrix(roll, pitch, yaw, False)
    quaternion_from_rotation_matrix = Quaternion(matrix=rot)
    #roll1, pitch1, yaw1 = rotation_matrix_to_euler(rot)

    # 步骤1：使用欧拉角构造四元数 (Z-Y-X 旋转顺序)
    q_yaw = Quaternion(axis=[0, 0, 1], angle=yaw)  # 绕Z轴
    q_pitch = Quaternion(axis=[0, 1, 0], angle=pitch)  # 绕Y轴
    q_roll = Quaternion(axis=[1, 0, 0], angle=roll)  # 绕X轴

    # 组合得到四元数
    q_combined = q_roll * q_pitch * q_yaw
    print("通过欧拉角组合的四元数:", q_combined)

    # 步骤2：通过四元数获取欧拉角
    yaw_extracted, pitch_extracted, roll_extracted = q_combined.yaw_pitch_roll
    print("通过四元数提取的欧拉角 (yaw, pitch, roll):", yaw_extracted, pitch_extracted, roll_extracted)

    # 验证欧拉角是否一致
    euler_angles_close = np.allclose([yaw, pitch, roll], [yaw_extracted, pitch_extracted, roll_extracted], atol=1e-6)
    print("欧拉角是否与原始一致:", euler_angles_close)
    import ipdb; ipdb.set_trace()


if __name__ == "__main__":
    test()
    quat2euler2romatrix()
    euler2quat2euler()
    quat2euler2quat()
    # # 示例：将欧拉角转换为四元数，再将四元数转换为欧拉角
    # roll, pitch, yaw = degrees_to_euler(45, 30, 60)
    # q = euler_to_quaternion(roll, pitch, yaw)
    # roll2, pitch2, yaw2 = quaternion_to_euler(q.w, q.x, q.y, q.z)
    # roll_deg, pitch_deg, yaw_deg = euler_to_degrees(roll2, pitch2, yaw2)
    # import ipdb; ipdb.set_trace()

    # roll, pitch, yaw = 0.5, 0.5, 0.5
    # euler_angles = (roll, pitch, yaw)
    # quaternion = euler_to_quaternion(*euler_angles)
    # yaw1, pitch1, roll1 = quaternion.yaw_pitch_roll
    # import ipdb; ipdb.set_trace()


 